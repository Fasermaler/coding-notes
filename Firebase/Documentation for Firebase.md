# Firebase

Firebase is a noSQL cloud database that is highly robust and allows for rapid prototyping of apps or scripts. There are 2 important parts to firebase, Cloud Firestore, Firebase Storage.



### Firebase Admin SDK

First thing you have to do after creating a firebase project is to navigate to **project overview ** > **settings ** > **service accounts** > **Firebase Admin SDK**. Then generate a new private key. You require this private key to access the firebase project server, so keep it well and never have it lying around in a public repo.

#### Initializing the JSON key

You can direct the script to your private key using the following:

```python
import firebase_admin
from firebase_admin import credentials

cred = credentials.Certificate("path/to/serviceAccountKey.json")
firebase_admin.initialize_app(cred)
```

This gives the script access to the server so that you can access it.



## Cloud Firestore

#### Creating a Project

1. Open the [Firebase Console](https://console.firebase.google.com/) and create a new project
2. In the **Database** section, click the **Create Database** button for Cloud Firestore.
3. Select a starting mode for your Cloud Firestore Security Rules:
   1. Test mode: For Web, IOS, or Android SDK
   2. Locked mode: For C#, Go, Java, Node.js, PHP, Python, or Ruby server client library

#### Install Firebase-Admin SDK 

Open Anaconda prompt (or use whatever IDE console you use) and enter the following:

```python
pip install --upgrade firebase-admin
```

#### Initialize a Firebase server 

To initialize a firebase server, locate the JSON key as previously mentioned and add the following snippet to your python script:

```python
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

# Use a service account
cred = credentials.Certificate('path/to/serviceAccount.json')
firebase_admin.initialize_app(cred)

db = firestore.client()
```

#### Adding Data

Cloud Firestore stores data in Documents, which are stored in Collections. 

Create a collection and document with the following coding:

```python
doc_ref = db.collection(u'users').document(u'degojix')
```

Set the document data with the following:

```python
doc_ref.set({
    u'first': u'dego',
    u'last': u'jix',
    u'born': 1897
})
```

This creates a collection called *users* with one document, *degojix*. The document *degojix* contains the information for the first and last names of *degojix* and the date of birth.

We can now add another document in the *users* collection called *fasermaler*. This time, we'll add another key-pair for *fasermaler* for a middle name.

```python
doc_ref = db.collection(u'users').document(u'fasermaler')
doc_ref.set({
    u'first': u'fa',
    u'middle': u'serma',
    u'last': u'ler',
    u'born': 1904
})
```

Notice that the key-pair for *middle* does not appear in *degojix*. You can think of collections as merely a folder, the documents themselves can store differing types of data with no concern for each other.

**Now when you appended the code and ran the whole script a second time, you might have discovered that firebase provided the following error:**

```python
ValueError: The default Firebase app already exists. This means you called initialize_app() more than once without providing an app name as the second argument. In most cases you only need to call initialize_app() once. But if you do want to initialize multiple apps, pass a second argument to initialize_app() to give each app a unique name.
```

This is because the initial firebase app (cred) is still in effect. You may open a new script  and use the following code instead:

```python
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

firebase_admin.get_app()
db = firestore.client()
```

The .get_app()  method will instruct the script to get the currently open firebase app (cred).

**ALTERNATIVELY**

Append the following line to the end of your code:

```python
firebase_admin.delete_app(firebase_admin.get_app())
```

This will get the app name and parse it into the delete_app() method to kill the app.

If for some reason you initialized an app without adding this line at the end of your code before running, you have to first run a separate instance of firebase that only consists of this line. Then proceed to append the line to the end of your existing code and run again. 

If you merely appended your code with a live firebase app already running, the code will fail before it reaches the .delete_app() method.

#### Reading Data

You can use the .get() method to retrieve the collection and read the data:

```python
users_ref = db.collection(u'users')
docs = users_ref.get()

for doc in docs:
    print(u'{} => {}'.format(doc.id, doc.to_dict()))
```

You might get something like the following output:

```bash
degojix => {'last': 'jix', 'first': 'dego', 'born': 1897}
fasermaler => {'last': 'ler', 'middle': 'serma', 'first': 'fa', 'born': 1904}
```



### Classes

You can even use custom classes with firebase documents! Here's an example:

```python
class City(object):
    def __init__(self, name, state, country, capital=False, population=0):
        self.name = name
        self.state = state
        self.country = country
        self.capital = capital
        self.population = population

    @staticmethod
    def from_dict(source):
        # ...

    def to_dict(self):
        # ...

    def __repr__(self):
        return u'City(name={}, country={}, population={}, capital={})'.format(
            self.name, self.country, self.population, self.capital)
    
    
city = City(name=u'Los Angeles', state=u'CA', country=u'USA')
db.collection(u'cities').document(u'LA').set(city.to_dict())
```





## Firebase Storage

There is a surprising lack of documentation for Firebase Storage, so not all of the following may be best practice, but most importantly **it works**. 

Special thanks to user [PhantomInsights](https://github.com/PhantomInsights) for their [documentation](https://github.com/PhantomInsights/firebase-python/tree/master/storage).

Unlike cloud firestore, which stores mainly strings of information, firebase storage allows for text files and image files to be edited (albeit in a less structured manner). How this is done that firebase storage bundles the file to be uploaded within a blob, which stores the file as well as metadata of the file. The blob contains the metadata in the form of a JSON, which can then be converted into dictionary to be easily manipulated by python.

#### Editing the Rules

Before we go on about how to upload files. First, go to your **firebase project** > **storage** on the left-hand bar. Then go over to rules. You should see the default rules are something like the following:

```perl
service firebase.storage {
  match /b/{bucket}/o {
    match /{allPaths=**} {
      allow read, write; if request.auth != null;
    }
  }
}
```

The **if request.auth != null;** line basically requires authentication to read and write to the bucket. For the time being, just remove this line. This allows anyone with access to the URL to read/write. Which is good for now since we are developing.

If for some reason, you would like to allow people to view files within the bucket but not write to it then remove **write**.

#### File Upload

The following code can be used for file upload:

```python
def upload_file():
    my_file = open("<FILE>", "rb")
    my_bytes = my_file.read()
    my_url = "https://firebasestorage.googleapis.com/v0/b/<PROJECT ID>appspot.com/o/<FOLDER>%2F<FILE>"
    my_headers = {"Content-Type": "<FILE TYPE"}
    my_request = urllib.request.Request(my_url, data=my_bytes, headers=my_headers, method="POST")
    try:
        loader = urllib.request.urlopen(my_request)   
    except urllib.error.URLError as e:
        message = json.loads(e.read())
        print(message["error"]["message"])
        image_available = 0
    else:
        print(loader.read())
```

The above code will upload a **<file>** to **<folder>** within your bucket in project **<project ID>**. The project ID can be obtained in your project information page within firebase console. Alternatively, you can get the address of the file and folder directly from the page if you have a file manually uploaded before.

After you uploaded the file, you might notice some output in the console (or if you went to the link within **my_url**), it would most likely appear like this:

```json
{
    "name": "<FILE>",
    "bucket": "<YOUR-PROJECT-ID>.appspot.com",
    "generation": "1537816796033378",
    "metageneration": "1",
    "contentType": "text/plain",
    "timeCreated": "2018-09-24T19:19:56.032Z",
    "updated": "2018-09-24T19:19:56.032Z",
    "storageClass": "STANDARD",
    "size": "10450",
    "md5Hash": "7aIjAPS+Sd0DaF5SmGTUYw==",
    "contentEncoding": "identity",
    "crc32c": "DObTDw==",
    "etag": "COLi7f6t1N0CEAE=",
    "downloadTokens": "c766b816-7429-4163-9e2d-1193e6f5ac78"
}
```

You might notice that this is basically a JSON. You might also notice that the URL does not point to the media itself but points to the JSON. This is the metadata for the file.

#### Retrieving Metadata

To retrieve metadata, use the following code:

```python
def retrieve_metadata(num):
    try:
        loader = urllib.request.urlopen("https://firebasestorage.googleapis.com/v0/b/<PROJECT ID>appspot.com/o/<FOLDER>%2F<FILE>")
    except urllib.error.URLError as e:
        message = json.loads(e.read())
        print(message["error"]["message"])
    else:
        print(loader.read())
       
```

You'll notice that we call the JSON library to read the output of the metadata. Also, notice that we accessed the upload URL but we have not go the file, instead we have only obtained the metadata for the file.

We can also parse the loader data through the .loads() method from the JSON library to get the function to return a python dictionary:

```python
def retrieve_metadata():

    try:
        loader = urllib.request.urlopen("https://firebasestorage.googleapis.com/v0/b/<PROJECT ID>appspot.com/o/<FOLDER>%2F<FILE>")
    except urllib.error.URLError as e:
        message = json.loads(e.read())
        print(message["error"]["message"])
        
    else:
        json_data = loader.read()
        load_d = json.loads(json_data)
        return load_d
```

So now if the output from retrieve_metadata is passed to a variable, you can access the values of the key-pairs easily.

```python
my_python_dict = retrieve_metadata()
print(my_python_dict['name'])
```

The console should then print out the file name. This makes it very easy to index the names of files into a list for easy reference and access.

#### Delete a File

The follow function deletes a file:

```python
def delete_file():
    my_url = "https://firebasestorage.googleapis.com/v0/b/<PROJECT ID>appspot.com/o/<FOLDER>%2F<FILE>"
    my_request = urllib.request.Request(my_url, method="DELETE")
    try:
        loader = urllib.request.urlopen(my_request)
    except urllib.error.URLError as e:
        message = json.loads(e.read())
        print(message["error"]["message"])
    else:
        print(loader.read())
```

This uses the delete method on the url. Removing the file from the bucket.

#### Updating Metadata

You can use the dump method to update the metadata of a file. The following function is how:

```python
def update_metadata():

    my_url = "https://firebasestorage.googleapis.com/v0/b/<PROJECT ID>appspot.com/o/<FOLDER>%2F<FILE>"
    my_headers = {"Content-Type": "application/json"}
    my_data = {"contentType": "application/binary"}
    json_data = json.dumps(my_data).encode()

    my_request = urllib.request.Request(my_url, data=json_data, headers=my_headers, method="PATCH")

    try:
        loader = urllib.request.urlopen(my_request)
    except urllib.error.URLError as e:
        message = json.loads(e.read())
        print(message["error"]["message"])
    else:
        print(loader.read())
```



#### Download File

Finally it's important to be able to download a file. The following code is for downloading image files.

```python
def download_file(num):
    my_url = "https://firebasestorage.googleapis.com/v0/b/<PROJECT ID>appspot.com/o/<FOLDER>%2F<FILE>"
    try:
        r = requests.get(my_url)
        with open(str(str(num) + ".jpg"), 'wb') as f:  
            f.write(r.content)
            f.close()
              
    except urllib.error.URLError as e:
        message = json.loads(e.read())
        print(message["error"]["message"])
    else:
        pass
```



