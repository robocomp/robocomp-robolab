import time
import Ice
import IceStorm
from rich.console import Console, Text
console = Console()
import os

slice_dir = os.path.dirname(os.path.abspath(__file__))
try:
    # Ice 3.8+
    Ice.loadSlice(["-I", slice_dir, os.path.join(slice_dir, "BatteryStatus.ice"), os.path.join(slice_dir, "Power.ice")])
except (TypeError):
    # Ice 3.7
    Ice.loadSlice(f'-I {slice_dir} {os.path.join(slice_dir, "BatteryStatus.ice")} {os.path.join(slice_dir, "Power.ice")}')
import RoboCompBatteryStatus
import RoboCompPower

class Powers(list):
    def __init__(self, iterable=list()):
        super(Powers, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompPower.PowerData)
        super(Powers, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompPower.PowerData)
        super(Powers, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompPower.PowerData)
        super(Powers, self).insert(index, item)
setattr(RoboCompPower, "Powers", Powers)


import batterystatusI
import powerI

class Publishes:
    def __init__(self, ice_connector:Ice.CommunicatorI, topic_manager, parameters):
        self.ice_connector = ice_connector
        self.mprx={}
        self.topic_manager = topic_manager


    def create_topic(self, property_name, topic_name, prefix, ice_proxy):
        topic = False
        topic_full_name = f"{prefix}/{topic_name}" if prefix else topic_name

        while not topic:
            try:
                topic = self.topic_manager.retrieve(topic_full_name)
            except IceStorm.NoSuchTopic:
                try:
                    console.log(f"{Text('WARNING', style='yellow')} {topic_full_name} topic did not create. {Text('Creating...', style='green')}")
                    topic = self.topic_manager.create(topic_full_name)
                except:
                    console.log(f"{Text('WARNING', style='yellow')}publishing the {topic_full_name} topic. It is possible that other component have created")

        pub = topic.getPublisher().ice_oneway()
        proxy = ice_proxy.uncheckedCast(pub)
        self.mprx[property_name] = proxy
        return proxy

    def get_proxies_map(self):
        return self.mprx


class Requires:
    def __init__(self, ice_connector:Ice.CommunicatorI, parameters):
        self.ice_connector = ice_connector
        self.mprx={}

    def get_proxies_map(self):
        return self.mprx

    def create_proxy(self, property_name, ice_proxy, proxy_string):
        try:
            base_prx = self.ice_connector.stringToProxy(proxy_string)
            proxy = ice_proxy.uncheckedCast(base_prx)
            self.mprx[property_name] = proxy
            return True, proxy
        
        except Ice.Exception as e:
            console.print_exception()
            console.log(f'Cannot get {property_name} property: {e}')
            self.status = -1
            return False, None


class Subscribes:
    def __init__(self, ice_connector:Ice.CommunicatorI, topic_manager, default_handler, parameters):
        self.ice_connector = ice_connector
        self.topic_manager = topic_manager

    def create_adapter(self, topic_name, prefix, interface_handler, endpoint_string):
        topic_full_name = f"{prefix}/{topic_name}" if prefix else topic_name

        adapter = self.ice_connector.createObjectAdapterWithEndpoints(topic_full_name, endpoint_string)
        handler = interface_handler
        proxy = adapter.addWithUUID(handler).ice_oneway()
        subscribe_done = False
        while not subscribe_done:
            try:
                topic = self.topic_manager.retrieve(topic_full_name)
                subscribe_done = True
            except Ice.Exception as e:
                try:
                    console.log(f"{Text('WARNING', style='yellow')} {topic_full_name} topic did not create. {Text('Creating...', style='green')}")
                    topic = self.topic_manager.create(topic_full_name)
                    subscribe_done = True
                except:
                    print(f"{Text('WARNING', style='yellow')}publishing the {topic_full_name} topic. It is possible that other component have created")
        qos = {}
        topic.subscribeAndGetPublisher(qos, proxy)
        adapter.activate()
        return adapter


class Implements:
    def __init__(self, ice_connector:Ice.CommunicatorI, default_handler, parameters):
        self.ice_connector = ice_connector

        self.batterystatus = self.create_adapter("BatteryStatus", batterystatusI.BatteryStatusI(default_handler, ""), parameters["Endpoints"]["BatteryStatus"])
        self.power = self.create_adapter("Power", powerI.PowerI(default_handler, ""), parameters["Endpoints"]["Power"])

    def create_adapter(self, property_name, interface_handler, endpoint_string):
        try:
            adapter = self.ice_connector.createObjectAdapterWithEndpoints(property_name, endpoint_string)
            adapter.add(interface_handler, Ice.stringToIdentity(property_name.lower()))
            adapter.activate()
            console.log(f"{property_name} adapter created in port {endpoint_string}")
        except Exception as e:
            console.print_exception()
            console.log(f"{Text('ERROR', style='red')} creating or activating adapter for {property_name}: {e}")
            self.status = -1


class InterfaceManager:
    def __init__(self, configData):
        props = Ice.createProperties()
        props.setProperty(
            "Ice.Warn.Connections",
            str(configData["Ice"]["Warn"]["Connections"])
        )
        props.setProperty(
            "Ice.Trace.Network",
            str(configData["Ice"]["Trace"]["Network"])
        )
        props.setProperty(
            "Ice.Trace.Protocol",
            str(configData["Ice"]["Trace"]["Protocol"])
        )
        props.setProperty(
            "Ice.MessageSizeMax",
            str(configData["Ice"]["MessageSizeMax"])
        )

        init_data = Ice.InitializationData()
        init_data.properties = props
        try:
            # Ice 3.8+
            self.ice_connector = Ice.initialize(initData=init_data)
        except (TypeError, ValueError): 
            # Ice 3.7
            self.ice_connector = Ice.initialize(init_data)
        self.status = 0

        needs_rcnode = False
        self.topic_manager = self.init_topic_manager(configData) if needs_rcnode else None

        self.requires = Requires(self.ice_connector, configData)
        self.publishes = Publishes(self.ice_connector, self.topic_manager, configData)
        self.implements = None
        self.subscribes = None

    def init_topic_manager(self, configData):
        obj = self.ice_connector.stringToProxy(configData["Proxies"]["TopicManager"])
        try:
            return IceStorm.TopicManagerPrx.checkedCast(obj)
        except Ice.ConnectionRefusedException as e:
            console.log(Text('Cannot connect to rcnode! This must be running to use pub/sub.', 'red'))
            self.status = -1
            exit(-1)

    def set_default_hanlder(self, handler, configData):
        self.implements = Implements(self.ice_connector, handler, configData)
        self.subscribes = Subscribes(self.ice_connector, self.topic_manager, handler, configData)

    def get_proxies_map(self):
        result = {}
        result.update(self.requires.get_proxies_map())
        result.update(self.publishes.get_proxies_map())
        return result

    def destroy(self):
        if self.ice_connector:
            self.ice_connector.destroy()




