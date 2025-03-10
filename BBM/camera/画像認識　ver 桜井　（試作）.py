import cv2
import supervision as sv
from ultralytics import YOLO
import numpy as np

# モデルの読み込み
model = YOLO("c:/Users/nagan/OneDrive - 埼玉大学/デスクトップ/パイソン/best.pt")
  # 事前に学習済みの重みファイルを指定 作成する必要あり

def process_webcam(output_file):
    # カメラのキャプチャを開始 
    cap = cv2.VideoCapture(0)  # 0はデフォルトのウェブカメラを指します

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # カメラの幅、高さ、フレームレートを取得
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    # ビデオライターの設定
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # コーデックの設定
    out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    bounding_box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # モデルでフレームを処理
        results = model(frame, conf=0.70)[0]
        detections = sv.Detections.from_ultralytics(results)

        # フレームに注釈を追加
        annotated_frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)

        # 注釈付きフレームをビデオファイルに書き込み
        out.write(annotated_frame)

        # フレームを表示
        cv2.imshow("Camera", annotated_frame)
        if cv2.waitKey(25) & 0xFF == ord("q"):
            break

    # リソースの解放
    cap.release()
    out.release()
    cv2.destroyAllWindows()

def webcam(output_file: str = "C:/Users/nagan/OneDrive - 埼玉大学/デスクトップ/パイソン/映像.mp4"):
    process_webcam(output_file)
 # 保存したいファイルの変更はここや！

if __name__ == "__main__":
    webcam()
