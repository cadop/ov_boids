import warp as wp
import numpy as np

from pxr import UsdGeom, Gf, Sdf, UsdShade, Vt
from omni.physx import get_physx_interface
import omni
import carb
import omni.timeline

from . import warpflight
from . import usd_utils

class Simulator():
    def __init__(self):

        self.separation_distance = 8.5
        self.alignment_distance = 25.5
        self.cohesion_distance = 80.5
        self.eccentricity_distance = 45.0

        self.S = 0.2
        self.K = 0.8
        self.M = 0.05
        self.X = 0.4

        self.obstacles = []
        self.device = 'cuda:0'

        self.agent_path = None
        self.instancer_path = '/World/Instancer'
        self.points_path = '/World/Points'


        # Should be somewhere between min and max perception radius
        self.hash_radius = 1.0

        self.agent_point_prim = None
        self.num_boids = 100

        self.instance_forward_vec = Gf.Vec3d(1,0,0)
        self.timeline_interface = omni.timeline.get_timeline_interface()

        self._simulation_event = None

        self.reset_params()
        # Set the forward vector for the boids


    def reset_params(self):

        # Stop the simulation playing to avoid changing array size during physics steps
        self.timeline_interface.stop()


        self.boid_positions = []
        # Randomly set initial positions in 3d space
        variable = 100
        for _ in range(    self.num_boids):
            self.boid_positions.append(np.array([np.random.uniform(-variable*3, variable), 
                                                np.random.uniform(-variable/5, variable/5), 
                                                np.random.uniform(-variable, variable)],
                                                dtype=float)
                                                )
                
        self.boid_positions = np.array(self.boid_positions, dtype=float)+ 0
        # self.boid_positions = Vt.Vec3fArray.FromNumpy(np.asarray(self.boid_positions,dtype=float))

        self.boid_velocities = []
        velocity_range = 14.5
        for _ in range(    self.num_boids):
            self.boid_velocities.append(np.array([np.random.uniform(-velocity_range/100, velocity_range), 
                                                np.random.uniform(-velocity_range/20.0, velocity_range/5.0), 
                                                np.random.uniform(-velocity_range/50.0, velocity_range/50.0)],
                                                dtype=float)
                                                )
        self.boid_velocities = np.array(self.boid_velocities, dtype=float)


        self.agents_radi = Vt.FloatArray.FromNumpy(np.array([1 for x in range(self.num_boids)],dtype=float))
        self.forces = [[0,0,0] for _ in range(self.num_boids)]
        self.leaders = [0 for _ in range(self.num_boids)]

        self.goal = np.array([20,50,0], dtype=float)

        self.boid_headings = []

        self._set_warp_params()
        self.config_hashgrid(self.num_boids)

    def _set_warp_params(self):

        # Numpy arrays
        
        self.boid_headings = np.asarray([np.array([0,0,0,1], dtype=float) for x in range(self.num_boids)])
        self.force_list = np.asarray([np.array([0,0,0], dtype=float) for x in range(self.num_boids)])

        # Initialize Warp Arrays
        self.agent_force_wp = wp.zeros(shape=self.num_boids,device=self.device, dtype=wp.vec3)
        self.boid_positions_wp = wp.array(self.boid_positions, device=self.device, dtype=wp.vec3)
        self.boid_velocities_wp = wp.array(self.boid_velocities, device=self.device, dtype=wp.vec3)
        self.agents_hdir_wp = wp.array(self.boid_headings, device=self.device, dtype=wp.vec4)

        # Initialize temp Warp Arrays
        self.xnew_wp = wp.zeros_like(wp.array(self.boid_positions, device=self.device, dtype=wp.vec3))
        self.vnew_wp = wp.zeros_like(wp.array(self.boid_positions, device=self.device, dtype=wp.vec3))
        self.hdir_wp = wp.zeros_like(wp.array(self.boid_headings, device=self.device, dtype=wp.vec4))

        self.leader_wp = wp.zeros(shape=self.num_boids,device=self.device, dtype=float)

        # Set parameters for warp struct
        self.wp_params = warpflight.Params()
        self.wp_params.s_d = self.separation_distance
        self.wp_params.a_d = self.alignment_distance
        self.wp_params.c_d = self.cohesion_distance
        self.wp_params.e_d = self.eccentricity_distance
        self.wp_params.S = self.S
        self.wp_params.K = self.K
        self.wp_params.M = self.M
        self.wp_params.X = self.X


    def register_simulation(self):
        self._callbacks()

    def _callbacks(self):
        self._simulation_event = get_physx_interface(
                                        ).subscribe_physics_step_events(
                                                                self._on_simulation_update)

    def _unregister(self):
        try:
            self._simulation_event.unsubscribe()
        except:
            self._simulation_event = None

    def _on_simulation_update(self, dt):
        if self.agent_point_prim is None:
            return 

        self._dt = dt
        self.run()

    def config_hashgrid(self, num_boids=None):
        '''Create a hash grid based on the number of agents
            Currently assumes z up

        Parameters
        ----------
        num_boids : int, optional
            _description_, by default None
        '''

        grid = int(np.sqrt(num_boids))
        self.grid = wp.HashGrid(dim_x=grid, dim_y=1, dim_z=grid, device=self.device)

    def integrate(self):
        # Given the forces, integrate for pos and vel
        wp.launch(kernel=warpflight.integrate,
                dim=self.num_boids,
                inputs=[self.boid_positions_wp, self.boid_velocities_wp, self.agent_force_wp, self._dt],
                outputs=[self.xnew_wp, self.vnew_wp],
                device=self.device
                )

        self.boid_positions_wp = self.xnew_wp
        self.boid_velocities_wp = self.vnew_wp

        self.boid_positions = self.boid_positions_wp.numpy()
        self.boid_velocities = self.boid_velocities_wp.numpy()
    
    def get_forces(self):
        '''compute forces for all boids'''

        self.grid.build(points=self.boid_positions_wp, radius=self.hash_radius)

        # launch kernel
        wp.launch(kernel=warpflight.get_forces,
                dim=self.num_boids,
                inputs=[self.boid_positions_wp, self.boid_velocities_wp, self._dt, self.grid.id, self.wp_params],
                outputs=[self.leader_wp, self.agent_force_wp],
                device=self.device
                )
        
        self.force_list = self.agent_force_wp.numpy()

    def set_heading(self, velocities):

        if self.boid_instancer is None:
            return

        # Only makes sense with instances and not geompoints
        up = wp.vec3(0.0,1.0,0.0)
        forward = wp.vec3(1.0,0.0,0.0)

        wp.launch(kernel=warpflight.heading,
                dim=self.num_boids,
                inputs=[self.boid_velocities_wp, up, forward],
                outputs=[self.hdir_wp],
                device=self.device
                )

        self.agents_hdir_wp = self.hdir_wp
        self.boid_headings = self.agents_hdir_wp.numpy()

        self.boid_instancer.GetOrientationsAttr().Set(self.boid_headings)


    def step(self, dt):
        '''integrate forces to update positions and velocities of boids'''

        # Set any of the new parameters
        self.wp_params.s_d = self.separation_distance
        self.wp_params.a_d = self.alignment_distance
        self.wp_params.c_d = self.cohesion_distance
        self.wp_params.e_d = self.eccentricity_distance
        self.wp_params.S = self.S
        self.wp_params.K = self.K
        self.wp_params.M = self.M
        self.wp_params.X = self.X
        
        # Get all agent forces
        self.get_forces()
        self.integrate()

        self.set_positions(self.boid_positions)
        self.set_heading(self.boid_velocities)

    def run(self):
        '''Runs the simulation for one step
        '''
        self.step(self._dt)


    def create_geompoints(self, stage_path=None, color=None):
        '''create and manage geompoints representing agents

        Parameters
        ----------
        stage_path : str, optional
            if not set, will use /World/Points, by default None
        color : (r,g,b), optional
            if not set, will make color red, by default None
        '''
        if stage_path: points_path = stage_path
        elif self.points_path: points_path = self.points_path
        else:          points_path = "/World/Points"

        if self.agent_path is None:
            return False


        self.stage = omni.usd.get_context().get_stage()
        self.agent_point_prim = UsdGeom.Points.Define(self.stage, points_path)
        self.agent_point_prim.CreatePointsAttr()

        width_attr = self.agent_point_prim.CreateWidthsAttr()
        width_attr.Set(self.agents_radi)

        self.agent_point_prim.CreateDisplayColorAttr()
        # For RTX renderers, this only works for UsdGeom.Tokens.constant
        color_primvar = self.agent_point_prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
        
        point_color = (1,0,0)
        color_primvar.Set([point_color])
        
        boid_positions = Vt.Vec3fArray.FromNumpy(np.asarray(self.boid_positions,dtype=float))

        
        ##### Create the instancer #####
        self.boid_instancer = usd_utils.create_boids_instancer(instance_path=self.instancer_path+"/PointInstancer", 
                                                               agent_instance_path=self.agent_path, 
                                                               nagents=self.num_boids, 
                                                               pos=self.boid_positions, 
                                                               radi=self.agents_radi, 
                                                               radius_max=1, 
                                                               forward_vec=self.instance_forward_vec)
        
        self.set_positions(boid_positions)


    def set_positions(self, positions):
        
        if self.agent_point_prim:
            self.agent_point_prim.GetPointsAttr().Set(positions)

        # Set the instancer positions
        if self.boid_instancer:
            self.boid_instancer.GetPositionsAttr().Set(positions)   

