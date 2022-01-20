import time
import Ice
import IceStorm
from rich.console import Console, Text
console = Console()


Ice.loadSlice("-I ./src/ --all ./src/CameraRGBDSimple.ice")
import RoboCompCameraRGBDSimple
Ice.loadSlice("-I ./src/ --all ./src/DifferentialRobot.ice")
import RoboCompDifferentialRobot
Ice.loadSlice("-I ./src/ --all ./src/Follower.ice")
import RoboCompFollower
Ice.loadSlice("-I ./src/ --all ./src/GenericBase.ice")
import RoboCompGenericBase
Ice.loadSlice("-I ./src/ --all ./src/HumanCameraBody.ice")
import RoboCompHumanCameraBody
Ice.loadSlice("-I ./src/ --all ./src/JointMotorSimple.ice")
import RoboCompJointMotorSimple
Ice.loadSlice("-I ./src/ --all ./src/SoundRotation.ice")
import RoboCompSoundRotation

class ImgType(list):
    def __init__(self, iterable=list()):
        super(ImgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(ImgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(ImgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(ImgType, self).insert(index, item)

setattr(RoboCompCameraRGBDSimple, "ImgType", ImgType)
class DepthType(list):
    def __init__(self, iterable=list()):
        super(DepthType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(DepthType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(DepthType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(DepthType, self).insert(index, item)

setattr(RoboCompCameraRGBDSimple, "DepthType", DepthType)
class DescriptorFloat(list):
    def __init__(self, iterable=list()):
        super(DescriptorFloat, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(DescriptorFloat, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, float)
        super(DescriptorFloat, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(DescriptorFloat, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "DescriptorFloat", DescriptorFloat)
class DescriptorByte(list):
    def __init__(self, iterable=list()):
        super(DescriptorByte, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(DescriptorByte, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(DescriptorByte, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(DescriptorByte, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "DescriptorByte", DescriptorByte)
class DescByteList(list):
    def __init__(self, iterable=list()):
        super(DescByteList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHumanCameraBody.DescriptorByte)
        super(DescByteList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHumanCameraBody.DescriptorByte)
        super(DescByteList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHumanCameraBody.DescriptorByte)
        super(DescByteList, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "DescByteList", DescByteList)
class DescFloatList(list):
    def __init__(self, iterable=list()):
        super(DescFloatList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHumanCameraBody.DescriptorFloat)
        super(DescFloatList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHumanCameraBody.DescriptorFloat)
        super(DescFloatList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHumanCameraBody.DescriptorFloat)
        super(DescFloatList, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "DescFloatList", DescFloatList)
class ImgType(list):
    def __init__(self, iterable=list()):
        super(ImgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(ImgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(ImgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(ImgType, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "ImgType", ImgType)
class GroundTruth(list):
    def __init__(self, iterable=list()):
        super(GroundTruth, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHumanCameraBody.TGroundTruth)
        super(GroundTruth, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHumanCameraBody.TGroundTruth)
        super(GroundTruth, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHumanCameraBody.TGroundTruth)
        super(GroundTruth, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "GroundTruth", GroundTruth)
class People(list):
    def __init__(self, iterable=list()):
        super(People, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHumanCameraBody.Person)
        super(People, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHumanCameraBody.Person)
        super(People, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHumanCameraBody.Person)
        super(People, self).insert(index, item)

setattr(RoboCompHumanCameraBody, "People", People)

import followerI



class Publishes:
    def __init__(self, ice_connector, topic_manager):
        self.ice_connector = ice_connector
        self.mprx={}
        self.topic_manager = topic_manager


    def create_topic(self, topic_name, ice_proxy):
        # Create a proxy to publish a AprilBasedLocalization topic
        topic = False
        try:
            topic = self.topic_manager.retrieve(topic_name)
        except:
            pass
        while not topic:
            try:
                topic = self.topic_manager.retrieve(topic_name)
            except IceStorm.NoSuchTopic:
                try:
                    topic = self.topic_manager.create(topic_name)
                except:
                    print(f'Another client created the {topic_name} topic? ...')
        pub = topic.getPublisher().ice_oneway()
        proxy = ice_proxy.uncheckedCast(pub)
        self.mprx[topic_name] = proxy
        return proxy

    def get_proxies_map(self):
        return self.mprx


class Requires:
    def __init__(self, ice_connector):
        self.ice_connector = ice_connector
        self.mprx={}

        self.CameraRGBDSimple = self.create_proxy("CameraRGBDSimpleProxy", RoboCompCameraRGBDSimple.CameraRGBDSimplePrx)

        self.DifferentialRobot = self.create_proxy("DifferentialRobotProxy", RoboCompDifferentialRobot.DifferentialRobotPrx)

        self.HumanCameraBody = self.create_proxy("HumanCameraBodyProxy", RoboCompHumanCameraBody.HumanCameraBodyPrx)

        self.JointMotorSimple = self.create_proxy("JointMotorSimpleProxy", RoboCompJointMotorSimple.JointMotorSimplePrx)

        self.SoundRotation = self.create_proxy("SoundRotationProxy", RoboCompSoundRotation.SoundRotationPrx)

    def get_proxies_map(self):
        return self.mprx

    def create_proxy(self, property_name, ice_proxy):
        # Remote object connection for
        try:
            proxy_string = self.ice_connector.getProperties().getProperty(property_name)
            try:
                base_prx = self.ice_connector.stringToProxy(proxy_string)
                proxy = ice_proxy.uncheckedCast(base_prx)
                self.mprx[property_name] = proxy
                return True, proxy
            except Ice.Exception:
                print('Cannot connect to the remote object (CameraSimple)', proxy_string)
                # traceback.print_exc()
                return False, None
        except Ice.Exception as e:
            console.print_exception(e)
            console.log(f'Cannot get {property_name} property.')
            return False, None


class Subscribes:
    def __init__(self, ice_connector, topic_manager, default_handler):
        self.ice_connector = ice_connector
        self.topic_manager = topic_manager

    def create_adapter(self, property_name, interface_handler):
        adapter = self.ice_connector.createObjectAdapter(property_name)
        handler = interface_handler
        proxy = adapter.addWithUUID(handler).ice_oneway()
        topic_name = property_name.replace('Topic','')
        subscribe_done = False
        while not subscribe_done:
            try:
                topic = self.topic_manager.retrieve(topic_name)
                subscribe_done = True
            except Ice.Exception as e:
                console.log("Error. Topic does not exist (creating)", style="blue")
                time.sleep(1)
                try:
                    topic = self.topic_manager.create(topic_name)
                    subscribe_done = True
                except:
                    console.log(f"Error. Topic {Text(topic_name, style='red')} could not be created. Exiting")
                    status = 0
        qos = {}
        topic.subscribeAndGetPublisher(qos, proxy)
        adapter.activate()
        return adapter


class Implements:
    def __init__(self, ice_connector, default_handler):
        self.ice_connector = ice_connector
        self.follower = self.create_adapter("Follower", followerI.FollowerI(default_handler))

    def create_adapter(self, property_name, interface_handler):
        adapter = self.ice_connector.createObjectAdapter(property_name)
        adapter.add(interface_handler, self.ice_connector.stringToIdentity(property_name.lower()))
        adapter.activate()


class InterfaceManager:
    def __init__(self, ice_config_file):
        # TODO: Make ice connector singleton
        self.ice_config_file = ice_config_file
        self.ice_connector = Ice.initialize(self.ice_config_file)
        needs_rcnode = False
        self.topic_manager = self.init_topic_manager() if needs_rcnode else None

        self.status = 0
        self.parameters = {}
        for i in self.ice_connector.getProperties():
            self.parameters[str(i)] = str(self.ice_connector.getProperties().getProperty(i))
        self.requires = Requires(self.ice_connector)
        self.publishes = Publishes(self.ice_connector, self.topic_manager)
        self.implements = None
        self.subscribes = None



    def init_topic_manager(self):
        # Topic Manager
        proxy = self.ice_connector.getProperties().getProperty("TopicManager.Proxy")
        obj = self.ice_connector.stringToProxy(proxy)
        try:
            return IceStorm.TopicManagerPrx.checkedCast(obj)
        except Ice.ConnectionRefusedException as e:
            console.log(Text('Cannot connect to rcnode! This must be running to use pub/sub.', 'red'))
            exit(-1)

    def set_default_hanlder(self, handler):
        self.implements = Implements(self.ice_connector, handler)
        self.subscribes = Subscribes(self.ice_connector, self.topic_manager, handler)

    def get_proxies_map(self):
        result = {}
        result.update(self.requires.get_proxies_map())
        result.update(self.publishes.get_proxies_map())
        return result

    def destroy(self):
        if self.ice_connector:
            self.ice_connector.destroy()




