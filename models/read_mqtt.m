function out_data = read_mqtt(address, topic)

persistent mqtt_comm
if isempty(mqtt_comm)
    mqtt_comm = MQTTComm(address, zeros(6,1), topic);
    out_data = zeros(6,1);
else
    out_data = mqtt_comm.ReadData();
end


end