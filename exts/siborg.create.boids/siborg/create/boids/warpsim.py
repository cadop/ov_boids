import warp as wp
import numpy as np

from pxr import UsdGeom, Gf, Sdf, UsdShade, Vt
from omni.physx import get_physx_interface
import omni
import carb

from . import warpflight
from . import usd_utils

class Simulator():
    def __init__(self):

        self.separation_distance = 1.0
        self.alignment_distance = 1.0
        self.cohesion_distance = 1.0
        eccentricity_distance = 1.0

        self.obstacles = []
        self.device = 'cuda:0'

        # Should be somewhere between min and max perception radius
        self.hash_radius = 1.0

        self.agent_point_prim = None
        self.num_boids = 120

        self.instance_forward_vec = Gf.Vec3d(1,0,0)
        self.reset_params()

        self._simulation_event = None

        # Set the forward vector for the boids


    def reset_params(self):
        self.boid_positions = []
        # Randomly set initial positions in 3d space
        variable = 20
        for _ in range(    self.num_boids):
            self.boid_positions.append(np.array([np.random.uniform(-variable*3, variable), 
                                                np.random.uniform(-variable/2, variable/2), 
                                                np.random.uniform(-variable*2, variable)],
                                                dtype=float)
                                                )
                
        self.boid_positions = np.array(self.boid_positions, dtype=float)+ 0
        # self.boid_positions = Vt.Vec3fArray.FromNumpy(np.asarray(self.boid_positions,dtype=float))

        self.boid_velocities = []
        velocity_range = 2
        for _ in range(    self.num_boids):
            self.boid_velocities.append(np.array([np.random.uniform(-velocity_range, velocity_range), 
                                                np.random.uniform(-velocity_range, velocity_range), 
                                                np.random.uniform(-velocity_range, velocity_range)],
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
        # self.boid_positions = np.asarray(self.boid_positions)
        # self.boid_velocities = np.asarray([np.array([0,0,0], dtype=float) for x in range(self.num_boids)])
        self.boid_headings = np.asarray([np.array([0,0,0,1], dtype=float) for x in range(self.num_boids)])
        self.force_list = np.asarray([np.array([0,0,0], dtype=float) for x in range(self.num_boids)])
        # self.agents_radi = np.random.uniform(self.radius_min, self.radius_max, self.num_boids)
        # self.agents_mass = [self.mass for x in range(self.num_boids)]
        # self.agents_percept = np.asarray([self.perception_radius for x in range(self.num_boids)])
        # self.agents_goal = np.asarray([np.array(self.goal, dtype=float) for x in range(self.num_boids)])
        # Initialize Warp Arrays
        self.agent_force_wp = wp.zeros(shape=self.num_boids,device=self.device, dtype=wp.vec3)
        self.boid_positions_wp = wp.array(self.boid_positions, device=self.device, dtype=wp.vec3)
        self.boid_velocities_wp = wp.array(self.boid_velocities, device=self.device, dtype=wp.vec3)
        self.agents_hdir_wp = wp.array(self.boid_headings, device=self.device, dtype=wp.vec4)
        # self.agents_goal_wp = wp.array(self.agents_goal, device=self.device, dtype=wp.vec3)
        # self.agents_radi_wp = wp.array(self.agents_radi, device=self.device, dtype=float)
        # self.agents_mass_wp = wp.array(self.agents_mass, device=self.device, dtype=float)
        # self.agents_percept_wp = wp.array(self.agents_percept, device=self.device, dtype=float)
        # Initialize temp Warp Arrays
        self.xnew_wp = wp.zeros_like(wp.array(self.boid_positions, device=self.device, dtype=wp.vec3))
        self.vnew_wp = wp.zeros_like(wp.array(self.boid_positions, device=self.device, dtype=wp.vec3))
        self.hdir_wp = wp.zeros_like(wp.array(self.boid_headings, device=self.device, dtype=wp.vec4))

        self.leader_wp = wp.zeros(shape=self.num_boids,device=self.device, dtype=float)

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
                inputs=[self.boid_positions_wp, self.boid_velocities_wp, self._dt, self.grid.id],
                outputs=[self.leader_wp, self.agent_force_wp],
                device=self.device
                )
        
        self.force_list = self.agent_force_wp.numpy()

    def set_heading(self, velocities):
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
        if stage_path: stage_loc = stage_path
        else:          stage_loc = "/World/Points"

        self.stage = omni.usd.get_context().get_stage()
        self.agent_point_prim = UsdGeom.Points.Define(self.stage, stage_loc)
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
        self.boid_instancer = usd_utils.create_boids_instancer(instance_path="/World/Bird/PointInstancer", 
                                                               agent_instance_path="/World/Bird/Agent", 
                                                               nagents=self.num_boids, 
                                                               pos=self.boid_positions, 
                                                               radi=self.agents_radi, 
                                                               radius_max=1, 
                                                               forward_vec=self.instance_forward_vec)
        
        self.set_positions(boid_positions)


    def set_positions(self, positions):
        
        self.agent_point_prim.GetPointsAttr().Set(positions)

        # Set the instancer positions
        self.boid_instancer.GetPositionsAttr().Set(positions)   
