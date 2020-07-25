import cv2
import os
import glob

image_folders = ['frames/rgb/', 'frames/dis_map/']
video_names = ['video_rgb.avi', 'video_dis_map.avi']

for i, image_folder in enumerate(image_folders):
    #images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    #frame = cv2.imread(os.path.join(image_folder, images[0]))
    images = [img for img in sorted(glob.glob(image_folder + '/*.png'), key=os.path.getmtime, reverse=False)]
    frame = cv2.imread(os.path.join('./', images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_names[i], 0, 1, (width,height))

    for image in images:
        #video.write(cv2.imread(os.path.join(image_folder, image)))
        video.write(cv2.imread(os.path.join('./', image)))

    cv2.destroyAllWindows()
    video.release()
