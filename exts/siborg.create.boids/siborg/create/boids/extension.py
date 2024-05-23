import omni.ext
import omni.ui as ui

# from . import core
from . import warpsim as core

 
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class SiborgCreateBoidsExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[siborg.create.boids] siborg create boids startup")

        self._count = 0

        self._window = ui.Window("Boids", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                label = ui.Label("boids")
                self.Sim = core.Simulator()

                def on_click():
                    if self.Sim == None:
                        self.Sim = core.Simulator()
                    self.Sim._unregister()
                    
                    self.Sim.register_simulation()

                def reset():
                    self.Sim.reset_params()

                def make_points():
                    self.Sim.create_geompoints()

                with ui.HStack():
                    ui.Button("make points", clicked_fn=make_points)
                    ui.Button("start", clicked_fn=on_click)
                    ui.Button("reset", clicked_fn=reset)

    def on_shutdown(self):
        print("[siborg.create.boids] siborg create boids shutdown")

        try: self.Sim._unregister()
        except: pass 

        try: self._on_update_sub.unsubscribe()
        except: self._on_update_sub = None 

        self.Sim._simulation_event = None

        self._window = None
        self.Sim = None