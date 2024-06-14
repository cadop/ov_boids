'''

https://www.cgtrader.com/free-3d-models/animals/bird/game-ready-bird-collection
'''

import omni.ext
import omni.ui as ui
import carb
from pxr import UsdGeom, Sdf

# from . import core
from . import warpsim as core
import os
from pathlib import Path
 
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class SiborgCreateBoidsExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[siborg.create.boids] siborg create boids startup")

        self._count = 0
        self.stage = omni.usd.get_context().get_stage()

        self._window = ui.Window("Boids", width=400, height=400)
        with self._window.frame:
            with ui.VStack():
                self.Sim = core.Simulator()

                def start():
                    if self.Sim == None:
                        self.Sim = core.Simulator()
                    self.Sim._unregister()
                    self.Sim.register_simulation()

                def reset(event=None, num_boids=None):
                    self.Sim.reset_params()
                    if num_boids != None:
                        self.Sim.num_boids = num_boids
                    self.Sim.reset_params()

                def update_num_boids(event=None, num_boids=None):
                    reset(event, num_boids)
                    make_points()

                def make_points():
                    self.Sim.create_geompoints()

                def assign_boid_path(event):
                    print(f'event: {event.mime_data}')
                    if event.mime_data == None or event.mime_data == '':
                        return
                    item = event.mime_data
                    # Check if its a prim, get its path
                    prim_path = Sdf.Path(item)
                    prim = self.stage.GetPrimAtPath(prim_path)
                    if prim.IsValid():
                        self.Sim.agent_path = item
                        self.asset_field.model.set_value(item)
                    elif item.endswith('.usd'):
                        # Get name of file using os and path
                        file_name = Path(item).stem
                        # This is external, add as a reference
                        new_prim_name = f'{self.stage.GetDefaultPrim().GetPath()}/{file_name}'
                        ref_prim = self.stage.OverridePrim(new_prim_name)
                        ref_prim.GetReferences().AddReference(item)             
                        self.Sim.agent_path = new_prim_name
                        self.asset_field.model.set_value(self.Sim.agent_path)
                    else:
                        self.asset_field.model.set_value('INVALID ITEM OR PATH')
                    
                def assign_instancer_path(event):
                    item = event.mime_data
                    self.Sim.instancer_path = item
                    self.instancer_field.model.set_value(item)

                def assign_points_path(event):
                    item = event.mime_data
                    self.Sim.points_path = item
                    self.points_field.model.set_value(item)

                with ui.HStack():
                    ui.Button("Populate", clicked_fn=make_points)
                    ui.Button("Start", clicked_fn=start)
                    ui.Button("Reset", clicked_fn=reset)
                with ui.HStack(height=30):
                    ui.Label("Boid Asset")
                    self.asset_field = ui.StringField(tooltip="asset")
                    self.asset_field.set_accept_drop_fn(lambda item: True)
                    self.asset_field.model.set_value(self.Sim.agent_path)
                    self.asset_field.set_drop_fn(assign_boid_path)
                with ui.HStack(height=30):
                    ui.Label("Instancer Parent")
                    self.instancer_field = ui.StringField(tooltip="instancer")
                    self.instancer_field.set_accept_drop_fn(lambda item: True)
                    self.instancer_field.model.set_value(self.Sim.instancer_path)
                    self.instancer_field.set_drop_fn(assign_instancer_path)
                with ui.HStack(height=30):
                    ui.Label("Points Parent")
                    self.points_field = ui.StringField(tooltip="points")
                    self.points_field.set_accept_drop_fn(lambda item: True)
                    self.points_field.model.set_value(self.Sim.points_path)
                    self.points_field.set_drop_fn(assign_points_path)
                with ui.HStack(height=30):
                    ui.Label("Number of Boids")
                    self.boids_field = ui.IntField(tooltip="boids")
                    self.boids_field.model.set_value(self.Sim.num_boids)
                    self.boids_field.model.add_value_changed_fn(lambda m : update_num_boids('num_boids', m.get_value_as_int()))

                # Allow custom settings for boids algorthim
                with ui.HStack(height=30):
                    ui.Label("Distances")

                    with ui.HStack(height=30):
                        ui.Label("Separation")
                        self.sep_field = ui.FloatDrag(tooltip="sep", min=0.1, max=200,step=0.1)
                        self.sep_field.model.set_value(self.Sim.separation_distance)
                        self.sep_field.model.add_value_changed_fn(lambda m : setattr(self.Sim, 'separation_distance', m.get_value_as_float()))

                    with ui.HStack(height=30):
                        ui.Label("Alignment")
                        self.align_field = ui.FloatDrag(tooltip="align", min=0.1, max=200,step=0.1)
                        self.align_field.model.set_value(self.Sim.alignment_distance)
                        self.align_field.model.add_value_changed_fn(lambda m : setattr(self.Sim, 'alignment_distance', m.get_value_as_float()))

                    with ui.HStack(height=30):
                        ui.Label("Cohesion")
                        self.coh_field = ui.FloatDrag(tooltip="coh", min=0.1, max=200,step=0.1)
                        self.coh_field.model.set_value(self.Sim.cohesion_distance)
                        self.coh_field.model.add_value_changed_fn(lambda m : setattr(self.Sim, 'cohesion_distance', m.get_value_as_float()))

                    with ui.HStack(height=30):
                        ui.Label("Eccentricity")
                        self.ecc_field = ui.FloatDrag(tooltip="ecc", min=0.1, max=200,step=0.1)
                        self.ecc_field.model.set_value(self.Sim.eccentricity_distance)
                        self.ecc_field.model.add_value_changed_fn(lambda m : setattr(self.Sim, 'eccentricity_distance', m.get_value_as_float()))

                with ui.HStack(height=30):
                    ui.Label("Multipliers")

                    with ui.HStack(height=30):
                        ui.Label("S")
                        self.s_field = ui.FloatDrag(tooltip="s", min=0.1, max=1,step=0.1)
                        self.s_field.model.set_value(self.Sim.S)
                        self.s_field.model.add_value_changed_fn(lambda m : setattr(self.Sim, 'S', m.get_value_as_float()))

                    with ui.HStack(height=30):
                        ui.Label("M")
                        self.m_field = ui.FloatDrag(tooltip="m", min=0.1, max=1,step=0.1)
                        self.m_field.model.set_value(self.Sim.M)
                        self.m_field.model.add_value_changed_fn(lambda m : setattr(self.Sim, 'M', m.get_value_as_float()))

                    with ui.HStack(height=30):
                        ui.Label("K")
                        self.k_field = ui.FloatDrag(tooltip="k", min=0.1, max=1,step=0.1)
                        self.k_field.model.set_value(self.Sim.K)
                        self.k_field.model.add_value_changed_fn(lambda m : setattr(self.Sim, 'K', m.get_value_as_float()))

                    with ui.HStack(height=30):
                        ui.Label("X")
                        self.x_field = ui.FloatDrag(tooltip="x", min=0.1, max=1,step=0.1)
                        self.x_field.model.set_value(self.Sim.X)
                        self.x_field.model.add_value_changed_fn(lambda m : setattr(self.Sim, 'X', m.get_value_as_float()))

                #TODO Obstacles


    def on_shutdown(self):
        print("[siborg.create.boids] siborg create boids shutdown")

        try: self.Sim._unregister()
        except: pass 

        try: self._on_update_sub.unsubscribe()
        except: self._on_update_sub = None 

        try: self.Sim.timeline_interface.release_timeline_interface()
        except: self.Sim.timeline_interface = None



        self.Sim._simulation_event = None

        self._window = None
        self.Sim = None