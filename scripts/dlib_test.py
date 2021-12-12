import os
import time

import face_recognition

# make a list of all the available images
start = time.time()
images = os.listdir('/home/erwin/catkin_ws/src/roas6000_final/images_match')

# load your image

image_to_be_matched = face_recognition.load_image_file('/home/erwin/catkin_ws/src/roas6000_final/test_data/t05.png')

# encoded the loaded image into a feature vector

image_to_be_matched_encoded = face_recognition.face_encodings(

    image_to_be_matched)[0]

# iterate over each image

for image in images:

    # load the image

    current_image = face_recognition.load_image_file('/home/erwin/catkin_ws/src/roas6000_final/images_match/'+ image)

    # encode the loaded image into a feature vector

    current_image_encoded = face_recognition.face_encodings(current_image)[0]

    # match your image with the image and check if it matches

    result = face_recognition.compare_faces(

        [image_to_be_matched_encoded], current_image_encoded)

    # check if it was a match

    if result[0] == True:

        print("Matched:" + image)

    else:

        print("Not matched:"+ image)

end = time.time()
print('time consumed is ' + str(end - start) + ' s')