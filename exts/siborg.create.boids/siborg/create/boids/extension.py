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

        self._window = ui.Window("Boids", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                label = ui.Label("boids")
                self.Sim = core.Simulator()

                def add_boid():
                    self.Sim.add_boid()

                def start():
                    if self.Sim == None:
                        self.Sim = core.Simulator()
                    self.Sim._unregister()
                    
                    self.Sim.register_simulation()

                def reset():
                    self.Sim.reset_params()

                def make_points():
                    self.Sim.create_geompoints()

                def accept_drops(event):
                    item = event.mime_data
                    # Check if its a prim, get its path
                    prim = self.stage.GetPrimAtPath(item)
                    if prim.IsValid():
                        self.asset_field.model.set_value(item)
                    elif item.endswith('.usd'):
                        # This is external, add as a reference
                        # Get name of file using os and path
                        file_name = Path(item).stem

                        ref_prim = self.stage.OverridePrim(f'{self.stage.GetDefaultPrim().GetPath()}/{file_name}')
                        ref_prim.GetReferences().AddReference(item)             
                        self.asset_field.model.set_value(f'{self.stage.GetDefaultPrim().GetPath()}/{file_name}')
                    else:
                        self.asset_field.model.set_value('INVALID ITEM OR PATH')
                    
                with ui.HStack():
                    ui.Button("add boid", clicked_fn=add_boid)
                    ui.Button("make points", clicked_fn=make_points)
                    ui.Button("start", clicked_fn=start)
                    ui.Button("reset", clicked_fn=reset)
                with ui.HStack():
                    ui.Label("Boid Asset")
                    self.asset_field = ui.StringField(tooltip="asset")
                    self.asset_field.set_accept_drop_fn(lambda item: True)
                    self.asset_field.set_drop_fn(accept_drops)


    def on_shutdown(self):
        print("[siborg.create.boids] siborg create boids shutdown")

        try: self.Sim._unregister()
        except: pass 

        try: self._on_update_sub.unsubscribe()
        except: self._on_update_sub = None 

        self.Sim._simulation_event = None

        self._window = None
        self.Sim = None