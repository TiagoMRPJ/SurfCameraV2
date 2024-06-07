from google.cloud import storage
from google.oauth2 import service_account

"""
This library exports functions to access Google Cloud Storage (GCS) buckets.
"""

import os
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/path/to/your/credentials.json"

class GoogleCloudAPI:
    def __init__(self):
        self.credentials = service_account.Credentials.from_service_account_file(
        'credentials.json')
        self.client = storage.Client()
        
    def list_buckets(self):
        """
        Returns a list of all bucket names.
        """
        buckets = self.client.list_buckets()
        bucket_names = []
        for bucket in buckets:
            bucket_names.append(bucket.name)
        return bucket_names
    
    def get_bucket(self, bucket_name):
        """
        Returns a bucket object from the given bucket name.
        """
        bucket = self.client.get_bucket(bucket_name)
        return bucket
    
    def create_bucket(self, bucket_name):
        """
        Creates a bucket with the given name.
        """
        bucket = self.client.create_bucket(bucket_name)
        return bucket

    def list_blobs(self, bucket_name):
        """
        Returns a list of all blob names in the given bucket.
        """
        bucket = self.get_bucket(bucket_name)
        blobs = bucket.list_blobs()
        blob_names = []
        for blob in blobs:
            blob_names.append(blob.name)
        return blob_names

    def download_blob(self, bucket_name, source_blob_name, destination_file_name):
        """
        Downloads a blob from the bucket to the given file.
        """
        bucket = self.get_bucket(bucket_name)
        blob = bucket.blob(source_blob_name)
        blob.download_to_filename(destination_file_name)

    def upload_blob(self, bucket_name, source_file_name, destination_blob_name):
        """
        Uploads a file to the bucket.
        """
        bucket = self.get_bucket(bucket_name)
        blob = bucket.blob(destination_blob_name)
        blob.upload_from_filename(source_file_name)

def main():
    GCAPI = GoogleCloudAPI()
    GCAPI.upload_blob('idmind-surf', 'sample1.avi', 'sample1.avi')

if __name__ == '__main__':
    main()
