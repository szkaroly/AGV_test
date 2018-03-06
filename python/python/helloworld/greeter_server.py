# Copyright 2015 gRPC authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""The Python implementation of the GRPC helloworld.Greeter server."""

from concurrent import futures
import time

import grpc

import helloworld_pb2
import helloworld_pb2_grpc

_ONE_DAY_IN_SECONDS = 60 * 60 * 24


class Greeter(helloworld_pb2_grpc.GreeterServicer):
    def __init__(self):
        self.myVariable = 10

    def getMyVar(self):
        return self.myVariable

    def setMyVar(self, val):
        self.myVariable = val

    def SayHello(self, request, context):
        print('I got a request from a client named:{0}'.format(request.name))
        return helloworld_pb2.HelloReply(message='Hello client! My Var is:{0}'.format(self.myVariable))

    def serve(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        helloworld_pb2_grpc.add_GreeterServicer_to_server(self, self.server)
        self.server.add_insecure_port('[::]:50051')
        self.server.start()

    def stop(self):
        self.server.stop(0)
