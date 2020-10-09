course_id = 'Master_Project'
github_repo = 'andresperez86/%s'%course_id
zip_file_url="https://github.com/%s/archive/master.zip"%github_repo

local_dir = "./local/"
dataset_dir = "./local/datasets/"

import requests, zipfile, io, os, shutil

# download library
def download_utils(force_download=False):

    if force_download or not os.path.exists("local"):

        dirname = course_id+"-master/"

        if os.path.exists(dirname):
            shutil.rmtree(dirname)
        
        r = requests.get(zip_file_url)
        z = zipfile.ZipFile(io.BytesIO(r.content))
        z.extractall()
        
        if os.path.exists("local"):
            shutil.rmtree("local")
        
        shutil.move(dirname+"/local", "local")
        shutil.rmtree(dirname)

# unzip the donloaded lead dataset
def unzip_dataset():

    with zipfile.ZipFile(dataset_dir+"Dataset_JPV.zip", 'r') as zip_ref:
        zip_ref.extractall(dataset_dir)
    
    os.remove(dataset_dir+"Dataset_JPV.zip")