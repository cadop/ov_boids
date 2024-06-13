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

        self._window = ui.Window("Boids", width=300, height=200)
        with self._window.frame:
            with ui.VStack():
                self.Sim = core.Simulator()

                def start():
                    if self.Sim == None:
                        self.Sim = core.Simulator()
                    self.Sim._unregister()
                    self.Sim.register_simulation()

                def reset():
                    self.Sim.reset_params()

                def make_points():
                    self.Sim.create_geompoints()

                def assign_boid_path(event):
                    item = event.mime_data
                    # Check if its a prim, get its path
                    prim = self.stage.GetPrimAtPath(item)
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
                    self.Sim.instancer_path = item
                    self.instancer_field.model.set_value(item)

                with ui.HStack():
                    ui.Button("Populate", clicked_fn=make_points)
                    ui.Button("Start", clicked_fn=start)
                    ui.Button("Reset", clicked_fn=reset)
                with ui.HStack(height=30):
                    ui.Label("Boid Asset")
                    self.asset_field = ui.StringField(tooltip="asset")
                    self.asset_field.set_accept_drop_fn(lambda item: True)
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

                #TODO Obstacles


    def on_shutdown(self):
        print("[siborg.create.boids] siborg create boids shutdown")

        try: self.Sim._unregister()
        except: pass 

        try: self._on_update_sub.unsubscribe()
        except: self._on_update_sub = None 

        self.Sim._simulation_event = None

        self._window = None
        self.Sim = None