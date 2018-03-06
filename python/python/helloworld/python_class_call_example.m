%pyversion 'C:\Program Files\Python36\python.exe'
% Needs a 3.6.x python installation (64 bit)
% pip install grpcio
% pip install grpcio-tools

clc; clear all;
display('Starting GRPC greeter service from python')
if count(py.sys.path,'') == 0
    insert(py.sys.path,int32(0),'');
end

py.importlib.import_module('greeter_client')

while true
py.greeter_client.run()
end