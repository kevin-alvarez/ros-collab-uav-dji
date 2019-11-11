import qrcode
import os
import json
import numpy as np
import matplotlib.pylab as plt

node_uav_1_port = 5001
node_uav_2_port = 5002
node_uav_3_port = 5003

def generate_qr_code():
  ros_ip = os.environ['ROS_IP']
  data_uav_1 = {
    "node_url" : "http://{}:{}".format(ros_ip, node_uav_1_port),
    "uav_id": 1
  }
  data_uav_2 = {
    "node_url" : "http://{}:{}".format(ros_ip, node_uav_2_port),
    "uav_id": 2
  }
  data_uav_3 = {
    "node_url" : "http://{}:{}".format(ros_ip, node_uav_3_port),
    "uav_id": 3
  }
  json_data_uav_1 = json.dumps(data_uav_1)
  json_data_uav_2 = json.dumps(data_uav_2)
  json_data_uav_3 = json.dumps(data_uav_3)
  qr_img_uav_1 = np.array(qrcode.make(json_data_uav_1), dtype=np.int32)
  qr_img_uav_2 = np.array(qrcode.make(json_data_uav_2), dtype=np.int32)
  qr_img_uav_3 = np.array(qrcode.make(json_data_uav_3), dtype=np.int32)
  fig, axes = plt.subplots(1, 3, figsize=(20, 10))
  ax = axes.ravel()
  ax[0] = plt.subplot(1, 3, 1)
  ax[0].imshow(qr_img_uav_1, cmap=plt.cm.gray)
  ax[0].axis('off')
  ax[0].title.set_text('UAV 1 QR sync')
  ax[1] = plt.subplot(1, 3, 2)
  ax[1].imshow(qr_img_uav_2, cmap=plt.cm.gray)
  ax[1].axis('off')
  ax[1].title.set_text('UAV 2 QR sync')
  ax[2] = plt.subplot(1, 3, 3)
  ax[2].imshow(qr_img_uav_3, cmap=plt.cm.gray)
  ax[2].axis('off')
  ax[2].title.set_text('UAV 3 QR sync')
  plt.suptitle('QR Codes for UAV sync with ROS')
  plt.show()

if __name__ == '__main__':
  generate_qr_code()

