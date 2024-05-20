import warp as wp
import numpy as np

from pxr import UsdGeom, Gf, Sdf, UsdShade, Vt
from omni.physx import get_physx_interface
import omni
import carb

from . import forces

class Simulator():
    def __init__(self):

        self.separation_distance = 1.0
        self.alignment_distance = 1.0
        self.cohesion_distance = 1.0
        eccentricity_distance = 1.0

        self.agent_point_prim = None
        self.num_boids = 70

        self.boid_positions = []
        # Randomly set initial positions in 3d space
        for _ in range(    self.num_boids):

            self.boid_positions.append(np.array([np.random.randint(-10, 10), 
                                                   np.random.randint(-10, 10), 
                                                   np.random.randint(-10, 10)],
                                                   dtype=float)
                                                   )
            
        self.boid_positions = np.array(self.boid_positions, dtype=float)+100
        # self.boid_positions = Vt.Vec3fArray.FromNumpy(np.asarray(self.boid_positions,dtype=float))

        self.boid_velocities = []
        for _ in range(    self.num_boids):

            self.boid_velocities.append(np.array([np.random.random()/3.0, 
                                                   np.random.random()/3.0, 
                                                   np.random.random()/3.0],
                                                   dtype=float)
                                                   )
        self.boid_velocities = np.array(self.boid_velocities, dtype=float)


        self.agents_radi = Vt.FloatArray.FromNumpy(np.array([1.0 for x in range(self.num_boids)],dtype=float))
        self.forces = [[0,0,0] for _ in range(self.num_boids)]
        self.leaders = [0 for _ in range(self.num_boids)]

        self._simulation_event = None


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

            s_i, k_i, m_i, l_i = forces.compute_forces(idx, 
                                               self.boid_positions, 
                                               self.boid_velocities, 
                                               self.leaders,
                                               self._dt
                                               )
                                               
            force.append(forces.set_forces(idx, 
                                           s_i,
                                           k_i, 
                                           m_i,  
                                           l_i)
                                           )

            # Update the leaders timers
            self.leaders[idx] = l_i

        self.forces = force


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
        self.set_positions(boid_positions)

    def set_positions(self, positions):
        
        self.agent_point_prim.GetPointsAttr().Set(positions)
