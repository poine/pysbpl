# Map class
import os
import cv2
import yaml

class Map:
    def __init__(self, **kwargs):
        if 'yaml_path' in kwargs:
            self.load_yaml(kwargs['yaml_path'])
            
    def load_yaml(self, yaml_path):
        print(' loading map from yaml {}'.format(yaml_path))
        with open(yaml_path, "r") as f:
            _yaml = yaml.safe_load(f)
            map_img_path = os.path.join(os.path.dirname(yaml_path), _yaml['image'])
            self.img = cv2.imread(map_img_path, 0)
            self.img[self.img==0] = 1
            self.img[self.img==255] = 0
            self.height, self.width = self.img.shape
            self.resolution = _yaml['resolution']
            self.origin = _yaml['origin']
            self.occupied_thresh = _yaml['occupied_thresh']
            self.free_thresh = _yaml['free_thresh']
