import rclpy
from rclpy.node import Node
from dr_interfaces.srv import SaveTraj
from dr_interfaces.srv import LoadTraj
from dr_interfaces.srv import ListTrajs
from dr_interfaces.srv import DeleteTraj
import os
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