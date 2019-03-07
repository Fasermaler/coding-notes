# -*- coding: utf-8 -*-
"""
Created on Tue Sep 25 02:52:51 2018

@author: Fasermaler
"""
import urllib
import requests
import json


def download_file(num):
    my_url = "" + str(num) + ".jpg?alt=media" # Insert project URL Accordingly
    try:
        #k = urllib.request.urlretrieve(my_url, str(str(num) + ".jpg"))
        r = requests.get(my_url)
        with open(str(str(num) + ".jpg"), 'wb') as f:  
            f.write(r.content)
            f.close()
              
    except urllib.error.URLError as e:
        message = json.loads(e.read())
        print(message["error"]["message"])
    else:
        pass
    
download_file(1)

