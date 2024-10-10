from ruamel.yaml import YAML 
from ruamel.yaml.comments import CommentedMap as OrderedDict
from pathlib import Path
import numpy as np

from guiLogic import landmarkType

class My_Yaml_Dump:
	def __init__(self, f) -> None:
		self.f = f


	def write(self, s):
		self.f.write(self.__clean_yaml(s.decode('utf-8')))


	def __clean_yaml(self, yaml_file: str) -> str:
		yaml_file = yaml_file.replace("'", "")
		return yaml_file

def stringToLandmarkType(string):
  if(string == "blue"):
    return landmarkType.BLUE
  elif(string == "yellow"):
    return landmarkType.YELLOW
  elif(string == "small-orange"):
    return landmarkType.ORANGE
  elif(string == "big-orange"):
    return landmarkType.BIG_ORANGE
  elif(string == "timekeeping"):
    return landmarkType.TIMEKEEPING
  elif(string == "invisible"):
     return landmarkType.INVISIBLE

  return landmarkType.UNDEFINED

def landmarkTypeToString(type):
  if(type == landmarkType.BLUE):
    return "0"
  elif(type == landmarkType.YELLOW):
    return "1"
  elif(type == landmarkType.ORANGE):
    return "2"
  elif(type == landmarkType.BIG_ORANGE):
    return "3"
  elif(type == landmarkType.TIMEKEEPING):
    return "4"
  elif(type == landmarkType.INVISIBLE):
    return "5"
  return "6"

def intToLandmarkType(int):
  if(int == 0):
    return "blue"
  elif(int == 1):
    return "yellow"
  elif(int == 2):
    return "small-orange"
  elif(int == 3):
    return "big-orange"
  elif(int == 4):
    return "timekeeping"
  return "unknown"

def writeYaml(fileName, cones, leftLane, rightLane, timeKeeping, startPose, earthToTrack):
    path = Path(fileName)

    all_cones = cones + leftLane + rightLane  + timeKeeping

    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z color score
SIZE 4 4 4 4 4
TYPE F F F I F
COUNT 1 1 1 1 1
WIDTH {len(all_cones)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(all_cones)}
DATA ascii
"""

    points = []
    for cono in all_cones:
        position = cono[0]
        cone_type = landmarkTypeToString(cono[1])
        # Format: "x y z color score"
        point_line = f"{position[0]} {position[1]} {int(0)} {cone_type} 1"
        points.append(point_line)

    with open(path, 'w') as f:
        f.write(header)
        f.write("\n".join(points))

def readYaml(fileName):
    path = Path(fileName)

    yaml=YAML(typ='safe')   
    data = yaml.load(path)

    unkownCones = []
    leftCones = []
    rightCones = []
    timekeeping = []
    lanesFirstWithLastConnected = False
    startPose = [np.zeros(3), np.zeros(3)]
    earthToTrack = [np.zeros(3), np.zeros(3)]
    if('lanesFirstWithLastConnected') in data['track']:
      lanesFirstWithLastConnected = bool(data['track']['lanesFirstWithLastConnected'])
    if('start') in data['track']:
      startPose = [np.array(data['track']['start']['position']), np.array(data['track']['start']['orientation'])]
    if('earthToTrack') in data['track']:
      earthToTrack = [np.array(data['track']['earthToTrack']['position']), np.array(data['track']['earthToTrack']['orientation'])]
    for c in data['track']['left']:
        leftCones.append([np.array(c['position']), c['class']])
    for c in data['track']['right']:
        rightCones.append([np.array(c['position']), c['class']])
    for c in data['track']['time_keeping']:
        # overwrite class as timekeeping
        timekeeping.append([np.array(c['position']), landmarkTypeToString(landmarkType.TIMEKEEPING)])
    for c in data['track']['unknown']:
        unkownCones.append([np.array(c['position']), c['class']])
    return (unkownCones, leftCones, rightCones, timekeeping, lanesFirstWithLastConnected, startPose, earthToTrack)

def readPCD(fileName):
    path = Path(fileName)
    
    with open(path, 'r') as f:
        lines = f.readlines()

    unknownCones = []
    leftCones = []
    rightCones = []
    timekeepingCones = []
    orangeCones = []
    
    data_section = False
    lanesFirstWithLastConnected = False
    startPose = [np.zeros(3), np.zeros(3)]  
    earthToTrack = [np.zeros(3), np.zeros(3)] 
    
    for line in lines:
        line = line.strip()
        
        if line.startswith("#") or not line:
            continue
        
        if line.startswith("DATA ascii"):
            data_section = True
            continue
        
        if data_section:
            parts = line.split()
            if len(parts) != 5:
                continue
            
            position = np.array([float(parts[0]), float(parts[1]), float(parts[2])])
            cone_type = float(parts[3])
            
            if cone_type == 0:
                leftCones.append([position, intToLandmarkType(cone_type)])
            elif cone_type == 1:
                rightCones.append([position, intToLandmarkType(cone_type)])
            elif cone_type == 2:
                orangeCones.append([position, intToLandmarkType(cone_type)])
            elif cone_type == 3:
                orangeCones.append([position, intToLandmarkType(cone_type)])
            elif cone_type == 4:
                timekeepingCones.append([position, intToLandmarkType(cone_type)])

    
    return (unknownCones, leftCones, rightCones, timekeepingCones, lanesFirstWithLastConnected, startPose, earthToTrack, orangeCones)