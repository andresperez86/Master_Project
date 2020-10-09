course_id = 'Master_Project'
session_id = 'TEST'
github_repo = 'andresperez86/%s'%course_id
zip_file_url="https://github.com/%s/archive/master.zip"%github_repo
#"https://github.com/andresperez86/Master_Project.git
#https://github.com/andresperez86/Master_Project/archive/master.zip
#https://raw.githubusercontent.com/andresperez86/Master_Project/master/init.py
#https://github.com/rramosp/2020.deeplearning/archive/master.zip
#https://github.com/andresperez86/Master_Project/blob/master/init.py
#endpoint = 'https://m5knaekxo6.execute-api.us-west-2.amazonaws.com/dev-v0001/rlxmooc'
#endpoint = 'http://localhost:5000/rlxmooc'

def get_last_modif_date(localdir):
    try:
        import time, os, pytz
        import datetime
        k = datetime.datetime.fromtimestamp(max(os.path.getmtime(root) for root,_,_ in os.walk(localdir)))
        localtz = datetime.datetime.now(datetime.timezone(datetime.timedelta(0))).astimezone().tzinfo
        k = k.astimezone(localtz)
        return k
    except Exception:
        return None
    
def init(force_download=False):

    if force_download or not os.path.exists("local"):
        print("replicating local resources")
        dirname = course_id+"-master/"
        if os.path.exists(dirname):
            shutil.rmtree(dirname)
        r = requests.get(zip_file_url)
        z = zipfile.ZipFile(io.BytesIO(r.content))
        z.extractall()
        if os.path.exists("local"):
            shutil.rmtree("local")
        shutil.move(dirname+"/content/local", "local")
        shutil.rmtree(dirname)

def get_weblink():
    from IPython.display import HTML
    return HTML("<h3>See <a href='"+endpoint+"/web/login' target='_blank'>my courses and progress</a></h2>")

def install_sourcedefender():
    print('enabling encryption...')
    output = subprocess.run(['pip', 'install', 'sourcedefender'], stderr=subprocess.PIPE)

    if output.returncode != 0:
        STDOUT_RED_COLOR = '\033[91m'
        STDOUT_RESET_COLOR = '\033[0m'
        print('Sourcedefender installation failed, returning')
        print(STDOUT_RED_COLOR + output.stderr.decode('ASCII') + STDOUT_RESET_COLOR)
    else:
       print('encryption enabled')


import requests, zipfile, io, os, shutil, subprocess
try:
    import sourcedefender
except ModuleNotFoundError:
    install_sourcedefender()
    import sourcedefender

