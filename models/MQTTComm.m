classdef MQTTComm < handle

properties
    jpos

    my_mqtt
    my_sub
end

methods
    function obj = MQTTComm(address, init_value, topic)
        if nargin==1
            obj.my_mqtt = mqtt(address, 'Port', 1883);
        elseif nargin==2
            obj.my_mqtt = mqtt(address, 'Port', 1883);
            obj.jpos = init_value;
        elseif nargin==3
            obj.my_mqtt = mqtt(address, 'Port', 1883);
%             obj.my_sub = subscribe(obj.my_mqtt, topic, 'Callback', @obj.ConvertMQTTData);
            obj.my_sub = subscribe(obj.my_mqtt, topic);
            obj.jpos = init_value;
        end
    end

    function ConvertMQTTData(obj, topic, msg)
        json_data = jsondecode(msg);
        obj.jpos = json_data.joint_actual_position;
        disp(datetime);
%         disp(msg);
    end
    
    function out_data = ReadData(obj)
        if obj.my_sub.MessageCount>0
            msg = read(obj.my_sub);
            json_data = jsondecode(msg);
            obj.jpos = json_data.joint_actual_position;
        end
        out_data = obj.jpos;
    end

    
    
    
end

end

