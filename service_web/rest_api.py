import router
import interfaces
import time

router_object = router.Router()

endpoint_name = router.generate_endpoint_name('rectDectApi', '0.1')
rectdect_interface_object = interfaces.get_interface_instance(router_object, 'service_control', endpoint_name)

router_object.start_route_incoming(thread_count = 1)

from flask_restplus import Api, Resource, fields
api = Api(version='0.1', title='RectDetect', description='An API for usage of RectDect robot skill')

ns_basic = api.namespace('basic', description='Basic skill operations')
ns_rectdetect = api.namespace('rectdetect', description='RectDect-specific operations')

@ns_basic.route('/initialize')
class Initialize(Resource):
    '''Initialize skill'''
    @ns_basic.doc('initialize_skill')
    def get(self):
        '''Initialize skill'''
        return rectdect_interface_object.initialize()

@ns_basic.route('/status')
class Status(Resource):
    '''Get skill status'''
    @ns_basic.doc('get_status')
    def get(self):
        '''Get skill status'''
        return rectdect_interface_object.status()

@ns_rectdetect.route('/detect_places')
class DetectPlaces(Resource):
    '''Detect places'''
    @ns_rectdetect.doc('detect_places')
    def get(self):
        '''Detect places'''
        return rectdect_interface_object.detectPlaces()

@ns_rectdetect.route('/detect_places_image')
class DetectPlaces(Resource):
    '''Detect places and get image'''
    @ns_rectdetect.doc('detect_places_image')
    def get(self):
        '''Detect places and get image'''
        return rectdect_interface_object.detectPlacesAndGetImage()
