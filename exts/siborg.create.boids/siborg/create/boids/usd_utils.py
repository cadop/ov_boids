from pxr import UsdGeom, Gf, Sdf, UsdShade, Usd
import omni


def create_boids_instancer(instance_path, 
                           agent_instance_path, 
                           nagents, 
                           pos, 
                           radi, 
                           radius_max, 
                           forward_vec):  
    
    stage = omni.usd.get_context().get_stage()


    prim = stage.GetPrimAtPath(agent_instance_path)
    if not prim.IsValid():
        return None

    point_instancer = UsdGeom.PointInstancer.Get(stage, instance_path)
    
    if not point_instancer:
        point_instancer = UsdGeom.PointInstancer(stage.DefinePrim(instance_path, "PointInstancer"))

    point_instancer.CreatePrototypesRel().SetTargets([agent_instance_path])
    proto_indices_attr = point_instancer.CreateProtoIndicesAttr()
    proto_indices_attr.Set([0] * nagents)

    ## max radius is scale of 1
    agent_scales = radi/radius_max
    agent_instancer_scales = [(x,x,x) for x in agent_scales] 

    # Set scale
    point_instancer.GetScalesAttr().Set(agent_instancer_scales)
    point_instancer.GetPositionsAttr().Set(pos)   
    # Set orientation
    rot = Gf.Rotation()
    rot.SetRotateInto(forward_vec, forward_vec)
    agent_headings =  [Gf.Quath(rot.GetQuat()) for x in range(nagents)] 
    point_instancer.GetOrientationsAttr().Set(agent_headings)


    # # Set start times
    start_times = [0.0, 1.0, 2.0, 3.0]  # Example start times for each instance
    # time_samples = Usd.TimeSamples(start_times)
    # point_instancer.GetTimeSamplesAttribute("positions").Set(time_samples, pos)
    # point_instancer.GetTimeSamplesAttribute("orientations").Set(time_samples, agent_headings)

    return point_instancer
