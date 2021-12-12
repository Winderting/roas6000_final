import os
# from tkinter import messagebox

import cv2
import numpy as np


def hog(img, bin_n=16):
    gx = cv2.Sobel(img, cv2.CV_32F, 1, 0)
    gy = cv2.Sobel(img, cv2.CV_32F, 0, 1)
    mag, ang = cv2.cartToPolar(gx, gy)
    bins = np.int32(bin_n*ang/(2*np.pi))    # quantizing binvalues in (0...16)
    bin_cells = bins[:10,:10], bins[10:,:10], bins[:10,10:], bins[10:,10:]
    mag_cells = mag[:10,:10], mag[10:,:10], mag[:10,10:], mag[10:,10:]
    hists = [np.bincount(b.ravel(), m.ravel(), bin_n) for b, m in zip(bin_cells, mag_cells)]
    hist = np.hstack(hists)     # hist is a 64 bit vector
    return hist


class FaceTrain:
    def __init__(self, class_name):
        self.class_name = class_name
        # self.xml_file = os.path.join(os.getcwd(), "../haarcascade_frontalface_default.xml")
        self.xml_file = os.path.join(os.getcwd(), "src/detection/haarcascade_frontalface_default.xml")


    # def face_detect(self, image):
    #     img = image.copy()
    #     img = cv2.resize(img,(512,512))
    #     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #     clf = cv2.CascadeClassifier(self.xml_file)
    #     faces = clf.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
    #     if len(faces) == 0:
    #         return None, None
    #     else:
    #         (x, y, w, h) = faces[0]
    #         return gray[y: y + w, x: x + h], faces[0]

    def prepare_training_data(self):
        training_data_dir = os.path.join(os.getcwd(), "src/detection/data")
        training_label_dir = os.path.join(os.getcwd(), "src/detection/label")
        # training_data_dir = os.path.join(os.getcwd(), "../data")

        dirs = os.listdir(training_data_dir)
        faces = []
        labels = []
        for dir_name in dirs:
            if not dir_name.startswith("f"):
                continue
            
            subject_dir_path = os.path.join(training_data_dir, dir_name)
            images = os.listdir(subject_dir_path)
            for image_name in images:
                if image_name.startswith("."):
                    continue
                img = cv2.imread(os.path.join(subject_dir_path, image_name))
                # face, rect = self.face_detect(img)
                img = cv2.resize(img, (512,512))

                h = img.shape[0]
                w = img.shape[1]
                image_txtname = image_name.split('.')[0]+'.txt'
                annotation = np.loadtxt(os.path.join(training_label_dir, image_txtname))
                label = int(annotation[0])
                xc = int(annotation[1]*w)
                yc = int(annotation[2]*h)
                bw = int(annotation[3]*w)
                bh = int(annotation[4]*h)
                x = xc-bw//2
                y = yc-bh//2
                
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                face = gray[x:x+bw, y:y+bh]
                # cv2.rectangle(img, (x, y), (x+bw, y+bh), (255, 0, 0), 2)
                # cv2.imshow("img",img)
                # cv2.waitKey(0)

                if face is not None:
                    faces.append(face)
                    labels.append(label)



        return faces, labels

    def main(self):
        # Preparing data...
        faces, labels = self.prepare_training_data()

        face_recognizer = cv2.face.createLBPHFaceRecognizer()
        face_recognizer.train(faces, np.array(labels))
        face_recognizer_file = os.path.join(
            os.getcwd(), "src/detection/face_recognizer_file.xml"
        )
        face_recognizer.save(face_recognizer_file)


        # svm = cv2.ml.SVM_create()
        # svm.setType(cv2.ml.SVM_C_SVC)
        # svm.setKernel(cv2.ml.SVM_LINEAR)
        # svm.setTermCriteria((cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))

        # face_feature = []
        # for face in faces:
        #     face_hog = hog(face)
        #     face_feature.append(face_hog)
        
        # face_feature = np.array(face_feature).astype('float32')
        # labels = np.reshape(np.array(labels),(-1,1))
        # svm.train(face_feature, cv2.ml.ROW_SAMPLE, labels)
        # recognizer_file = os.path.join(
        #     os.getcwd(), "src/detection/svm_drecognizer_file.dat"
        # )

        # svm.save(recognizer_file)

        print('Training successful')


if __name__ == "__main__":
    cus_train = FaceTrain('face')
    cus_train.main()

    classes_dir = os.path.join(
            os.getcwd(), "src/detection/label/classes.txt"
        )
    f = open(classes_dir,"r")
    classes = f.readlines()
    print(classes[0])