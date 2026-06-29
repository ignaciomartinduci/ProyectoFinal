# Copyright 2026 Ignacio Martín Duci
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import rclpy
from rclpy.node import Node
from dr_interfaces.srv import SaveTraj
from dr_interfaces.srv import LoadTraj
from dr_interfaces.srv import ListTrajs
from dr_interfaces.srv import DeleteTraj
import os
import sys
import json

class StorageNode(Node):

    def __init__(self):

        super().__init__("storage_node")

        self.TRAJECTORIES_PATH = os.path.expanduser('~/ros2_PFE/src/trajectories')
        os.makedirs(self.TRAJECTORIES_PATH, exist_ok=True)

        self.save_srv  = self.create_service(SaveTraj,  '/dr/save_traj',  self.save_traj_callback)
        self.load_srv  = self.create_service(LoadTraj,  '/dr/load_traj',  self.load_traj_callback)
        self.delete_srv = self.create_service(DeleteTraj, '/dr/delete_traj', self.delete_traj_callback)
        self.list_srv  = self.create_service(ListTrajs, '/dr/list_trajs', self.list_trajs_callback)
        if os.environ.get('DR_DEBUG') == '1':
            print('\033[92mNODO storage_node INICIADO CORRECTAMENTE\033[0m  srvs: /dr/save_traj | /dr/load_traj | /dr/delete_traj | /dr/list_trajs', file=sys.stderr)

    def save_traj_callback(self, request, response):
        try:
            name = request.name
            version = 1
            while os.path.exists(os.path.join(self.TRAJECTORIES_PATH, f'{name}_v{version}.json')):
                version += 1
            name = f'{name}_v{version}'
            filepath = os.path.join(self.TRAJECTORIES_PATH, f'{name}.json')
            data = {
                'name':   name,
                'points': request.points,
                'q_traj': list(request.q_traj)
            }
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            response.success = True
            response.message = f'Trayectoria {name} guardada.'
            self.get_logger().info(response.message)
            if os.environ.get('DR_DEBUG') == '1':
                G, R = '\033[92m', '\033[0m'
                print(f'{G}--> [storage_node]{R} save: \'{name}\' guardada ({request.points} puntos)', file=sys.stderr)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def load_traj_callback(self, request, response):
        try:
            name = request.name
            filepath = os.path.join(self.TRAJECTORIES_PATH, f'{name}.json')
            if not os.path.exists(filepath):
                version = 1
                while os.path.exists(os.path.join(self.TRAJECTORIES_PATH, f'{name}_v{version}.json')):
                    version += 1
                latest = version - 1
                if latest < 1:
                    raise FileNotFoundError(f'No se encontró ninguna trayectoria con nombre {name}.')
                name = f'{name}_v{latest}'
                filepath = os.path.join(self.TRAJECTORIES_PATH, f'{name}.json')
            with open(filepath, 'r') as f:
                data = json.load(f)
            response.q_traj = data['q_traj']
            response.points = data['points']
            response.success = True
            response.message = f'Trayectoria {name} cargada.'
            self.get_logger().info(response.message)
            if os.environ.get('DR_DEBUG') == '1':
                G, R = '\033[92m', '\033[0m'
                print(f'{G}--> [storage_node]{R} load: \'{name}\' cargada ({response.points} puntos)', file=sys.stderr)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.q_traj = []
            response.points = 0
        return response

    def list_trajs_callback(self, request, response):
        try:
            files = os.listdir(self.TRAJECTORIES_PATH)
            names = [f.replace('.json', '') for f in files if f.endswith('.json')]
            response.names = names
            response.success = True
            if os.environ.get('DR_DEBUG') == '1':
                G, R = '\033[92m', '\033[0m'
                print(f'{G}--> [storage_node]{R} list: {len(names)} trayectorias encontradas', file=sys.stderr)
        except Exception as e:
            response.names = []
            response.success = False
        return response

    def delete_traj_callback(self, request, response):
        try:
            filepath = os.path.join(self.TRAJECTORIES_PATH, f'{request.name}.json')
            if os.path.exists(filepath):
                os.remove(filepath)
                response.success = True
                response.message = f'Trayectoria {request.name} eliminada.'
                self.get_logger().info(response.message)
                if os.environ.get('DR_DEBUG') == '1':
                    G, R = '\033[92m', '\033[0m'
                    print(f'{G}--> [storage_node]{R} delete: \'{request.name}\' eliminada', file=sys.stderr)
            else:
                response.success = False
                response.message = f'Trayectoria {request.name} no encontrada.'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def main(args=None):

    rclpy.init(args=args)
    node = StorageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()