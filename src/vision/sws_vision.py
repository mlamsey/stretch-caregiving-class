import cv2
import os

def images_to_video(dir_path, fps=25):
    image_folder = dir_path
    # video_name = 'test_raw.mp4'
    video_name = 'rendered.avi'

    images = sorted([img for img in os.listdir(image_folder) if img.endswith(".png")])
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # video = cv2.VideoWriter(os.path.join(image_folder, video_name), fourcc, fps=fps, frameSize=(width,height))
    fourcc = 0
    video = cv2.VideoWriter(os.path.join(image_folder, video_name), fourcc, fps=fps, frameSize=(width,height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()

def render_mocap(video_path):
    pass

if __name__ == '__main__':
    images_to_video('/home/hello-robot/test/rendered')