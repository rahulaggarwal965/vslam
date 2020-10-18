import os
import subprocess
from threading import Thread
from time import sleep

from flask import Flask, make_response, render_template, flash, request, redirect, url_for, send_from_directory, Request
from werkzeug.utils import secure_filename
from werkzeug.exceptions import default_exceptions, HTTPException, InternalServerError

from flask_sqlalchemy import SQLAlchemy

import cv2

UPLOADS_FOLDER = 'static/uploads/'

# CHANGE
ALLOWED_EXT = {'mp4'}

app = Flask(__name__)
app.config['UPLOADS_FOLDER'] = UPLOADS_FOLDER
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///visual-slammers.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
app.secret_key = b'_flklmlfw\n.xwdw/'
# CHANGE
PORT=69

db = SQLAlchemy(app)

class Uploads(db.Model):
        _id = db.Column('id', db.Integer, primary_key=True)
        video_path = db.Column(db.String(100))
        point_cloud_map_path = db.Column(db.String(100))
        camera_poses_path = db.Column(db.String(100))
        thumbnail_path = db.Column(db.String(100))
        finalized = db.Column(db.Boolean, default=False)

"""
Returns: int - the unique id of the newly inserted upload
Post condition: upload is inserted into db and changes are commited
"""
def db_insert_upload(video_extension):
        new_upload = Uploads()
        db.session.add(new_upload)
        db.session.commit()

        str_new_upload_id = str(new_upload._id)

        new_upload.video_path = os.path.join(app.config['UPLOADS_FOLDER'], str_new_upload_id + '/video.' + video_extension)
        new_upload.point_cloud_map_path = os.path.join(app.config['UPLOADS_FOLDER'], str_new_upload_id + '/point_cloud_map.ply')
        new_upload.camera_poses_path = os.path.join(app.config['UPLOADS_FOLDER'], str_new_upload_id + '/camera_poses.ply')
        new_upload.thumbnail_path = os.path.join(app.config['UPLOADS_FOLDER'], str_new_upload_id + '/thumbnail.png')

        db.session.commit()

        return str_new_upload_id

def debug_print_all_entries():
        print('Here is what is in the Uploads table:')
        for each in Uploads.query.all():
                print(each._id, each.video_path, each.point_cloud_map_path, \
                each.camera_poses_path, each.thumbnail_path, each.finalized)
        print()

def debug_delete_all_entries():
        if input('Are you sure you want to delete everything in the database? y:n ') != 'y':
                print('NOT DELETED')
                return
        Uploads.query.delete()
        db.session.commit()
        print('DELETED')

debug_print_all_entries()

# Uploads.query.filter_by(_id=2).first().finalized = True
# db.session.commit()

# Testing db stuff
# print(db_insert_upload(), '----this is the uid')
# debug_print_all_entries()

"""
Helpful SQL stuff.
"""
# Select:
# Uploads.query.filter_by(id=something).first()
# Limit query:
# Uploads.query.limit(8).all()
# Delete:
# Uploads.query.filter_by(id=something).first().delete()
# for tmp_ in Uploads.query.filter_by(id=something):
#         temp_.delete()
# Update:
# just query an upload and then do that_upload.{columnname} = new_val

def allowed_file(filename):
        return '.' in filename and \
                filename.rsplit('.', 1)[1].lower() in ALLOWED_EXT


@app.route('/', methods=['GET'])
def index():
        return render_template("index.html")

@app.route('/upload', methods=['GET', 'POST'])
def upload():
        if request.method == 'POST':
                # check if the post request has the video part
                if 'video' not in request.files:
                        return redirect(request.url)

                video = request.files['video']

                # if user does not select file, browser also
                # submit an empty part without filename
                if video.filename == '':
                        return error('ERROR: You must choose a video to upload.')

                if not video and allowed_file(video.filename):
                        return error('ERROR: Invalid video or video extension.')

                # Adds a new database entry
                output_ID = db_insert_upload(video_extension=video.filename.rsplit('.', 1)[1].lower())

                str_output_ID = str(output_ID)
                output_dir = os.path.join(app.config['UPLOADS_FOLDER'], str_output_ID)

                # Deals with duplicate random ids (should never happen)
                if os.path.exists(output_dir):
                        os.rmdir(output_dir)
                        os.mkdir(output_dir)
                else:
                        os.mkdir(output_dir)
                        # save images to file system
                        video.save(os.path.join(output_dir, 'video.mp4'))

                # Get a thumbnail and write to filesystem

                vidcap = cv2.VideoCapture(os.path.join(output_dir, 'video.mp4'))
                success, image = vidcap.read()
                if not success:
                        success, image = vidcap.read()
                        if not success:
                                success, image = vidcap.read()
                if success:                        
                        cv2.imwrite(os.path.join(output_dir, 'thumbnail.png'), image)
                else:
                        # Give up, just give them a default thumbnail...
                        this_upload = Uploads.query.filter_by(_id=id).first()
                        this_upload.thumbnail_path = "static/default_thumbnail.png"
                        db.session.commit()

                def do_after_return(id):
                        # Temporary to show that threading works
                        sleep(10)

                        # Call VSLAM here

                        #recon = os.system("python openMVG/openMVG_Build/software/SfM/SfM_GlobalPipeline.py {} {}".format(outputDir, outputDir+"/recon"))


                        # Update filesystem and DB...
                        this_upload = Uploads.query.filter_by(_id=id).first()
                        # ...
                        this_upload.finalized = True
                        db.session.commit()
                        print('MADE FINALIZED')

                thread = Thread(target=do_after_return, args=output_ID)
                thread.start()
                print('SENT HTML')
                
                return gallery(output_ID)
        
        return render_template("upload.html")

# Only should display finished results
@app.route('/gallery', methods=['GET'])
@app.route('/gallery/<int:id>', methods=['GET'])
def gallery(id=None):
        if id != None:
                this_upload = Uploads.query.filter_by(_id=id).first()
                
                if this_upload == None:
                        return error('ERROR: No such gallery id: ' + str(id))

                # If we haven't finished running SLAM and updating database
                if not this_upload.finalized:
                        return render_template("processing.html", id=id)

                video_path = this_upload.video_path
                point_cloud_map_path = this_upload.point_cloud_map_path
                camera_poses_path = this_upload.camera_poses_path

                return render_template("result.html", id=id, video_path=video_path, point_cloud_map_path=point_cloud_map_path, camera_poses_path=camera_poses_path)

        finalized = list(Uploads.query.filter_by(finalized=True))
        # Get newest first
        finalized.reverse()

        return render_template("gallery.html", finalized=finalized)



def error(message, code=400):
    return render_template("error.html", code=code, message=message)

def errorhandler(e):
    if not isinstance(e, HTTPException):
        e = InternalServerError()
    return error(e.name, e.code)

# Listen for errors
for code in default_exceptions:
    app.errorhandler(code)(errorhandler)


# Just in case database and tables aren't already setup
db.create_all()

if __name__ == '__main__':
        app.run(debug=True, host='0.0.0.0', port=PORT, use_debugger=False)
        # CHANGE DEBUG IN PRODUCTION!
