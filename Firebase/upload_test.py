# -*- coding: utf-8 -*-
"""
Created on Tue Sep 25 01:05:35 2018

@author: Fasermaler
"""
import pickle
import firebase_admin
import datetime
import requests
import urllib
import time
from datetime import date
from firebase_admin import credentials
from firebase_admin import firestore
from google.cloud import storage


try:
    firebase_admin.delete_app(firebase_admin.get_app())
except:
    pass

# Use a service account
cred = credentials.Certificate('') # Add JSON key accordingly
firebase_admin.initialize_app(cred)

firebase_admin.get_app()
db = firestore.client()

for i in range(1, 2):
# test image available variable
    image_available = 0

#                
    
    def upload_file():
        global image_available
        image_url = "%2Ftemp_img_1" #Append projecy URL accordingly
        data = pickle.dumps(cv2.imread('1.jpg'))
        files = {'file':'1.data'}
        with open('1.pkl', 'wb') as outfile:
            outfile.write(data)
            
        files = {'1.pkl', open('1.pkl', 'wb')}
        try:
            my_request = requests.post(url=image_url, files=files)
            image_available = 1
        except:
            print("Failed to get file from RPI 1 at: " + str(datetime.datetime.now()))
            image_available = 0 
    upload_file()        
 
    doc_ref = db.collection(u'RPIs').document(u'1')
    doc_ref.set({
            u'1 rpi': u'1', 
            u'2 date': str(date.today()),
            u'3 time': str(datetime.datetime.now().time()),
            u'4 imagestate': u'available'
        })
    if image_available == 1:
        doc_ref.update({
            u'4 imagestate': u'available'
            })
    elif image_available == 0:
        doc_ref.update({
            u'4 imagestate': u'unavailable'
        })
    time.sleep(1)
        


firebase_admin.delete_app(firebase_admin.get_app())