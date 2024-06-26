import warp as wp
import numpy as np

from pxr import UsdGeom, Gf, Sdf, UsdShade, Vt
from omni.physx import get_physx_interface
import omni
import carb

from . import forces
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

        self.agent_path = None
        self.instancer_path = '/World/Instancer'
        self.points_path = '/World/Points'

        self.agent_point_prim = None
        self.num_boids = 100

        self.instance_forward_vec = Gf.Vec3d(-1,0,0) 
        self.timeline_interface = omni.timeline.get_timeline_interface()

        self._simulation_event = None

        self.reset_params()

        # Set the forward vector for the boids


    def reset_params(self):
        self.timeline_interface.stop()

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
        velocity_range = 10
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

    def integrate(self, x, v, f, dt):
        ''' take position, velocity, force, and dt to compute updated position and velocity '''
        v1 = v + ( (f * 1.0) * dt ) # new velocity
        x1 = x + (v1 * dt) # new position

        return x1, v1
    
    def get_forces(self):
        '''compute forces for all boids'''
        force = []

        for idx in range(self.num_boids):
            # continue
        
            s_i, k_i, m_i, o_i, l_i = forces.compute_forces(idx, 
                                               self.boid_positions, 
                                               self.boid_velocities, 
                                               self.leaders,
                                               self.obstacles,
                                               self._dt
                                               )
            
        
            force.append(forces.set_forces(idx, 
                                           s_i,
                                           k_i, 
                                           m_i,
                                           o_i,   
                                           l_i)
                                           )

            # Update the leaders timers
            self.leaders[idx] = l_i

        self.forces = force


    def set_heading(self, velocities):
        # Only makes sense with instances and not geompoints
        if self.boid_instancer is None:
            return

        self.boid_headings = []
        rot = Gf.Rotation()
        for vel in velocities:
            normalized_vel = vel / np.linalg.norm(vel)  # Normalize the velocity
            tovec = Gf.Vec3d(tuple(normalized_vel))
            rot.SetRotateInto(self.instance_forward_vec, tovec)
            self.boid_headings.append(Gf.Quath(rot.GetQuat()))

        self.boid_instancer.GetOrientationsAttr().Set(self.boid_headings)


    def step(self, dt):
        '''integrate forces to update positions and velocities of boids'''

        # Get all agent forces
        self.get_forces()

        for i in range(self.num_boids):
            
            x, v, f = self.boid_positions[i], self.boid_velocities[i], self.forces[i]
            x1, v1 = self.integrate(x, v, f, dt)
            self.boid_positions[i] = x1
            self.boid_velocities[i] = v1

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

        self.reset_params()
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

