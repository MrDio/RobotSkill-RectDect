import router
import api
import interfaces

router_object = router.Router()

import pickle


# Skill-specific required packages
import numpy as np
#import pyrealsense2 as rs
import cv2
import imutils
import base64

# Skill-specific helper functions
def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

def base64_to_image(uri):
    encoded_data = uri.split(',')[1]
    nparr = np.fromstring(base64.b64decode(encoded_data), np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    return img

endpoint_name = router.generate_endpoint_name('cameraApi', '0.1')
camera_api = interfaces.get_interface_instance(router_object, 'service-camera', endpoint_name, connection_to_base_service = True)


class RectDetect_API_Class:
    def __init__(self):
        self.camera_id = None
        self.camera_is_ready = False
        self.max_distance = None
        self.min_distance = None
        self.initialized = False
        self.found_objects = []

        # Constants and variables of skill
        self.classes = {
            "ObenLinks": 0.69,
            "ObenRechts": 0.28,
            "UntenLinks": 0.56,
            "UntenRechts": 0.51
        }
        self.classDistances = {
            "ObenLinks": {
                "ObenRechts": 0.1202,
                "UntenLinks": 0.17265,
                "UntenRechts": 0.19727
            },
            "ObenRechts": {
                "ObenLinks": 0.1202,
                "UntenLinks": 0.2242,
                "UntenRechts": 0.17465
            },
            "UntenLinks": {
                "ObenLinks": 0.17265,
                "ObenRechts": 0.2242,
                "UntenRechts": 0.1201
            },
            "UntenRechts": {
                "ObenLinks": 0.19727,
                "ObenRechts": 0.17465,
                "UntenLinks": 0.1201
            },
        }
        self.found_objects = {
            "ObenLinks": [],
            "ObenRechts": [],
            "UntenLinks": [],
            "UntenRechts": []
        }


    def status(self):
        return { "is_camera_ready": self.camera_is_ready}

    def initialize(self):
        print('In function')
        connected_devices = camera_api.get_connected_devices()
        print(connected_devices)
        if (len(connected_devices["devices"]) == 0):
            return {"status" : 'no_camera_available'}
        self.camera_id = connected_devices["devices"][0]["serial_number"]

        create_result = camera_api.create_pipeline(self.camera_id)
        if (create_result["status"] != 'created'):
            return {"status" : 'device_not_initialized'}

        color_stream_config = {'height' : 480, 'width' : 640, 'framerate' : 30}
        depth_stream_config = {'height' : 480, 'width' : 640, 'framerate' : 30}

        setup_result = camera_api.setup_stream(self.camera_id, color_stream_config, depth_stream_config)
        if (setup_result["status"] != 'stream_configured'):
            return {"status" : 'device_not_initialized'}

        start_result = camera_api.start(self.camera_id)
        if (start_result["status"] != 'pipeline_started'):
            return {"status" : 'device_not_initialized'}
        else:
            self.camera_is_ready = True

        self.depth_scale = camera_api.get_scale(self.camera_id, from_depth_sensor = True)

        self.max_distance = 1 / self.depth_scale["depth"]
        self.min_distance = 0.2 / self.depth_scale["depth"]

        self.initialized = True

        return {'status' : 'initialized'}

    def detectPlaces(self):
        if (self.initialized == True):
            depth_frame = None
            color_frame = None

            frames = camera_api.get_frames(self.camera_id, get_color_frame = True, get_depth_frame = True)

            color_frame = pickle.loads(frames["color_frame"])
            depth_frame = pickle.loads(frames["depth_frame"])

            if ((depth_frame is None) or (color_frame is None)):
                return {"status" : 'not_all_required_images_available'}

            intrinsics = camera_api.get_intrinsics(self.camera_id, from_image_frame = True, from_depth_frame = True)


    		# Get intrinsic matrices for calculate world coordinates
            depth_intrin = intrinsics["depth_intrinsics"]
            color_intrin = intrinsics["color_intrinsics"]

    		#depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)


    		# Convert images to numpy arrays
            # Info: Convertion already done in the camera api - so just store frames
            depth_image = depth_frame
            color_image = color_frame

    		# Filter unintresting parts of image
            depth_image[depth_image > self.max_distance] = 0
            depth_image[depth_image < self.min_distance] = 0

    		# Actuall processing of the image
            gray = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
            gray = adjust_gamma(gray, 3)
            blurred = cv2.bilateralFilter(gray, 3, 5,5)
            # Edge detection
            edges = cv2.Canny(blurred, 64, 192)

            # Post adjustments befor classification
            for i in range(10):
                edges = cv2.GaussianBlur(edges, (3,3),0)

    		# Detect contours
            cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)

            found_objects_tmp = self.found_objects.copy()
            for x in self.classes:
                found_objects_tmp[x] = []

    		#Determine all possible objects
            for c in cnts:
    			# Get minimal Rectangle to enclose the contour
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

    			# Calculate center of contour
                result = box.sum(axis=0)/4
                cX = int(result[0])
                cY = int(result[1])

                if (rect[1][1] != 0):
    				# Feature we try to classify with
                    sides = (rect[1][0], rect[1][1])
                    longerSide = max(sides)
                    shorterSide = min(sides)
                    ratio = round(shorterSide / longerSide, 2)

    				# Is the ratio close to one of the classes?
                    for x in self.classes:
    					# Maybe parameterize tolerance 0.03
                        if (abs(self.classes[x]-ratio) < 0.03):
                            depth = depth_image[cY][cX]

                            result_point = camera_api.deproject_pixel_to_point(self.camera_id, [cX, cY], depth*self.depth_scale["depth"])
                            pos_3d = result_point["point"]

                            if (pos_3d[0] == 0.0) and (pos_3d[1] == 0.0) and (pos_3d[2] == 0.0):
                                continue

                            found_objects_tmp[x].append([
                                (cX,cY),
                                box,
                                (pos_3d[0], pos_3d[1], pos_3d[2]),
                                0
                            ])

    				# Voting for the possible real objects
                    for x in self.classes:
                        for i in range(len(found_objects_tmp[x])):
                            remain_classes = self.classes.copy()
                            remain_classes.pop(x)
                            for y in remain_classes:
                                for j in range(len(found_objects_tmp[y])):
                                    # Calculate distance between these objects
                                    obj_x = found_objects_tmp[x][i]
                                    obj_y = found_objects_tmp[y][j]

                                    vec2d_x = np.asarray(obj_x[2][:2])
                                    vec2d_y = np.asarray(obj_y[2][:2])
                                    dist = np.linalg.norm(vec2d_x-vec2d_y)
    								# print("Distance is "+str(dist))
    								# Obj of x need a specific distance to obj of y
    								#print("Checking object at " + str(obj_x[0]) + " of "+ x +" with object at " + str(obj_y[0]) +" of "+ y)
                                    if (abs(self.classDistances[x][y] - dist) < 0.01):
                                        obj_x[3] = obj_x[3] + 1

            c_validatedObjects = 0
            final_objects = {}

            for x in self.classes:
    			#print("For "+x+" we found "+ str( sum(1 for i in found_objects_tmp[x] if i[3] > 0) ) + " items")
                if (len(found_objects_tmp[x]) > 0):
                    obj = max(found_objects_tmp[x], key=lambda x: x[3])
                    if (obj[3] > 0):
                        c_validatedObjects += 1
                        final_objects[x] = obj[2]

            return { "found": c_validatedObjects, "objects": final_objects }
        else:
            return {'status' : 'not_initialized'}

    def detectPlacesAndGetImage(self):
        if (self.initialized == True):
            depth_frame = None
            color_frame = None

            frames = camera_api.get_frames(self.camera_id, get_color_frame = True, get_depth_frame = True)

            color_frame = pickle.loads(frames["color_frame"])
            depth_frame = pickle.loads(frames["depth_frame"])

            if ((depth_frame is None) or (color_frame is None)):
                return {"status" : 'not_all_required_images_available'}

            intrinsics = camera_api.get_intrinsics(self.camera_id, from_image_frame = True, from_depth_frame = True)

    		# Get intrinsic matrices for calculate world coordinates
            depth_intrin = intrinsics["depth_intrinsics"]
            color_intrin = intrinsics["color_intrinsics"]

    		#depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

    		# Convert images to numpy arrays
            # Info: Convertion already done in the camera api - so just store frames
            depth_image = depth_frame
            color_image = color_frame

    		# Filter unintresting parts of image
            depth_image[depth_image > self.max_distance] = 0
            depth_image[depth_image < self.min_distance] = 0

    		# Actuall processing of the image
            gray = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
            gray = adjust_gamma(gray, 3)
            blurred = cv2.bilateralFilter(gray, 3, 5,5)
    		# Edge detection
            edges = cv2.Canny(blurred, 64, 192)

    		# Post adjustments befor classification
            for i in range(10):
                edges = cv2.GaussianBlur(edges, (3,3),0)

    		# Detect contours
            cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)

            found_objects_tmp = self.found_objects.copy()
            for x in self.classes:
                found_objects_tmp[x] = []

            #Determine all possible objects
            for c in cnts:
                # Get minimal Rectangle to enclose the contour
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

    			# Calculate center of contour
                result = box.sum(axis=0)/4
                cX = int(result[0])
                cY = int(result[1])

                if (rect[1][1] != 0):
    				# Feature we try to classify with
                    sides = (rect[1][0], rect[1][1])
                    longerSide = max(sides)
                    shorterSide = min(sides)
                    ratio = round(shorterSide / longerSide, 2)

                    # Is the ratio close to one of the classes?
                    for x in self.classes:
                        # Maybe parameterize tolerance 0.03
                        if abs(self.classes[x]-ratio) < 0.03:
                            depth = depth_image[cY][cX]

                            result_point = camera_api.deproject_pixel_to_point(self.camera_id, [cX, cY], depth*self.depth_scale["depth"])
                            pos_3d = result_point["point"]

                            if (pos_3d[0] == 0.0) and (pos_3d[1] == 0.0) and (pos_3d[2] == 0.0):
                                continue

                            found_objects_tmp[x].append([
                                (cX,cY),
                                box,
                                (pos_3d[0], pos_3d[1], pos_3d[2]),
                                0
                            ])

    				# Voting for the possible real objects
                    for x in self.classes:
                        for i in range(len(found_objects_tmp[x])):
                            remain_classes = self.classes.copy()
                            remain_classes.pop(x)
                            for y in remain_classes:
                                for j in range(len(found_objects_tmp[y])):
                                    # Calculate distance between these objects
                                    obj_x = found_objects_tmp[x][i]
                                    obj_y = found_objects_tmp[y][j]

                                    vec2d_x = np.asarray(obj_x[2][:2])
                                    vec2d_y = np.asarray(obj_y[2][:2])
                                    dist = np.linalg.norm(vec2d_x-vec2d_y)
    								# print("Distance is "+str(dist))
    								# Obj of x need a specific distance to obj of y
    								#print("Checking object at " + str(obj_x[0]) + " of "+ x +" with object at " + str(obj_y[0]) +" of "+ y)
                                    if abs(self.classDistances[x][y] - dist) < 0.01:
                                        obj_x[3] = obj_x[3] + 1

            c_validatedObjects = 0
            final_objects = {}

            for x in self.classes:
    			#print("For "+x+" we found "+ str( sum(1 for i in found_objects_tmp[x] if i[3] > 0) ) + " items")
                if (len(found_objects_tmp[x]) > 0):
                    obj = max(found_objects_tmp[x], key=lambda x: x[3])
                    if (obj[3] > 0):
                        c_validatedObjects += 1
                        final_objects[x] = obj[2]
                        color_image = cv2.drawContours(color_image,[obj[1]],0,(0,0,255),2)
                        cv2.circle(color_image, obj[0], 2, (0,0,255), 10)
                        cv2.putText(color_image, x, (obj[0][0], obj[0][1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            img = cv2.imencode(".png", color_image)[1].tostring()
            img_b64 = base64.b64encode(img)

            return { "found": c_validatedObjects, "objects": final_objects, "image": img_b64.decode('utf-8') }
        else:
            return {'status' : 'not_initialized'}

endpoint_name = router.generate_endpoint_name('rectDetectApi', '0.1')
api_object = api.register_api_endpoint(router_object, endpoint_name, RectDetect_API_Class(), use_threaded_request_processing = True)
router_object.start_route_incoming(thread_count = 1, keep_attached = True)
