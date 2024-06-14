'''
Hartman, Christopher, and Bedrich Benes. “Autonomous Boids.” 
Computer Animation and Virtual Worlds, vol. 17, no. 3-4, 2006, pp. 199-206, doi:10.1002/cav.123.

Many things are not clear in the paper, so modifications may have been done. 
'''

import warp as wp

@wp.struct
class Params:
    s_d: float  = 8.5   #4.5
    c_d: float = 80.5  #10.5
    a_d: float  = 25.5    #20.0
    e_d: float = 45.0    #5.0
    S: float = 0.2
    K: float = 0.8
    M: float = 0.05
    X: float = 0.4

@wp.kernel
def get_forces(positions: wp.array(dtype=wp.vec3), 
               velocities: wp.array(dtype=wp.vec3),
               dt: float,
               grid : wp.uint64, 
               params: Params,
               leader: wp.array(dtype=float),
               forces: wp.array(dtype=wp.vec3), 
               ):

    # thread index
    tid = wp.tid()

    s_d, c_d, a_d, e_d = params.s_d, params.c_d, params.a_d, params.e_d
    
    # Current boids position
    pi = positions[tid]
    vi = velocities[tid]
    l_i = leader[tid]

    # Separation distance


    _force, _l_i = compute_force(pi, vi, positions, velocities, s_d, c_d, a_d, e_d, dt, l_i, grid, params)

    # Update leader status
    leader[tid] = _l_i
    forces[tid] = _force

@wp.kernel
def integrate(x : wp.array(dtype=wp.vec3),
                v : wp.array(dtype=wp.vec3),
                f : wp.array(dtype=wp.vec3),
                dt: float,
                xnew: wp.array(dtype=wp.vec3),
                vnew: wp.array(dtype=wp.vec3), 
            ):
    
    tid = wp.tid()

    x0 = x[tid]
    v0 = v[tid]
    f0 = f[tid]

    v1 = v0 + (f0*1.0) * dt
    x1 = x0 + v1 * dt

    xnew[tid] = x1
    vnew[tid] = v1

    # Limit velocity
    max_speed = 80.0
    vnew[tid] = wp.normalize(v1) * wp.min(wp.length(v1), max_speed)



@wp.kernel
def heading(v : wp.array(dtype=wp.vec3),
            up : wp.vec3, 
            forward : wp.vec3,
            hdir: wp.array(dtype=wp.vec4), 
            ):
    
    tid = wp.tid()
    v0 = v[tid]
    vnorm = wp.normalize(v0)

    hdir[tid] = velocity_to_quaternion(up, forward, vnorm)


@wp.func
def velocity_to_quaternion(up : wp.vec3, 
                           forward : wp.vec3, 
                           velocity: wp.vec3):
    # Construct a quaternion that rotates the agent's forward direction to align with the velocity vector
    if wp.length(forward) > 0: forward = wp.normalize(forward)
    if wp.length(velocity) > 0: velocity = wp.normalize(velocity)
    else: 
        velocity = forward

    dot = wp.dot(forward, velocity) # Clip the dot product to avoid numerical instability
    if dot == 1.0:
        # If the forward and velocity vectors are already aligned, return the identity quaternion
        return wp.vec4(0.0, 0.0, 0.0, 1.0)
    else:
        axis = wp.cross(forward, velocity)
        if wp.length(axis) > 0.0: axis = wp.normalize(axis)  # Normalize the axis of rotation
        else:axis = up  # Use a default axis of rotation if the iwput is a zero vector
        angle = wp.acos(dot)  # Calculate the angle of rotation with clipping
        
        qw = wp.cos(angle/2.0)  # Calculate the scalar component of the quaternion
        qx = wp.sin(angle/2.0) * axis[0]  # Calculate the vector component of the quaternion
        qy = wp.sin(angle/2.0) * axis[1]  # Calculate the vector component of the quaternion
        qz = wp.sin(angle/2.0) * axis[2]  # Calculate the vector component of the quaternion
        
        return wp.vec4(qx, qy, qz, qw)
    

@wp.func
def separation(p: wp.vec3, 
               pn: wp.array(dtype=wp.vec3),  
               s_d: float, 
               grid: wp.uint64,
               ):
    ''' Separation force '''

    s_i = wp.vec3(0.0, 0.0, 0.0)

    query = wp.hash_grid_query(grid, p, s_d)
    index = int(0)

    while(wp.hash_grid_query_next(query, index)):
        j = index
        neighbor = pn[j]

        # compute distance to neighbor point
        dist = wp.length(p-neighbor)
        if (dist <= s_d):

            # Steerforce
            s_i += p - pn[j]

    return s_i

@wp.func
def cohesion(p: wp.vec3, 
               pn: wp.array(dtype=wp.vec3),  
               c_d: float, 
               grid: wp.uint64,
               ):
    
    k_i = wp.vec3(0.0, 0.0, 0.0)
    c_i = wp.vec3(0.0, 0.0, 0.0)

    query = wp.hash_grid_query(grid, p, c_d)
    index = int(0)

    count = wp.float(0.0)
    while(wp.hash_grid_query_next(query, index)):
        j = index
        neighbor = pn[j]

        # compute distance to neighbor point
        dist = wp.length(p-neighbor)
        if (dist <= c_d):

            # Steerforce
            c_i += pn[j]
            count += 1.0

    c_i = c_i / (count)
    k_i = c_i - p

    return k_i

@wp.func
def alignment(p: wp.vec3, 
               vn: wp.array(dtype=wp.vec3),  
               a_d: float, 
               grid: wp.uint64,
               ):

    m_i = wp.vec3(0.0, 0.0, 0.0)

    query = wp.hash_grid_query(grid, p, a_d)
    index = int(0)

    count = wp.float(0.0)
    while(wp.hash_grid_query_next(query, index)):
        j = index
        neighbor = vn[j]

        # compute distance to neighbor point
        dist = wp.length(p-neighbor)
        if (dist <= a_d):
            # Check if velocity is too high
            m_i += vn[j]
            count += 1.0

    if count == 0: return wp.vec3(0.0, 0.0, 0.0)

    m_i = m_i / count

    return m_i


@wp.func
def eccentricity(p: wp.vec3, 
                 pn: wp.array(dtype=wp.vec3), 
                 e_d: float,
                 grid: wp.uint64,
                 ):
    
    # Distance boid can see
    e = e_d

    c_i = wp.vec3(0.0, 0.0, 0.0)
    count = wp.float(0.0)

    query = wp.hash_grid_query(grid, p, e_d)
    index = int(0)

    while(wp.hash_grid_query_next(query, index)):
        j = index
        neighbor = pn[j]

        # compute distance to neighbor point
        dist = wp.length(p-neighbor)
        if (dist <= e_d):
            c_i += pn[j]
            count += 1.0

    # Center of Density
    c_i = c_i / count
    # Distance to the center of density
    cl_i = wp.length(p - c_i)

    x_i = cl_i / e

    return x_i

@wp.func
def leadership(v: wp.vec3, 
               k_i: wp.vec3, 
               x_i: float, 
               l_i: float, 
               X: float, 
               dt: float,
               ):
    ''' 
    Not clear what the paper is explaining with the algorithm steps. Here is the guess:

    - We need to decide when the bird becomes a leader, and after that, 
    how long does it stay a leader. 
    - when timer is 0, the bird is not a leader
    - the timer stops birds from becoming leaders when they are leaders

    dt is timestep, currently 1 is used to start the leader countdown
    '''

    # If currently and still is a leader, apply the time-based leadership force
    if l_i > 0.0:
        return l_i - dt
        
    # Check where the bird is in relation to the flock
    _v_i = wp.normalize(v)
    _k_i = wp.normalize(k_i)
    _l_i = wp.dot(_k_i, _v_i)

    # If the dot product is negative, the bird is behind the flock, and should not be be a leader
    if _l_i < 0: 
        return -1.0

    # Trigger runaway
    rg = wp.rand_init(1111)
    gauss_rand = wp.randf(rg)
    if (x_i >= X*gauss_rand): 
        return 1.0
   
    return -1.0


def obstacle(idx, obstacles, velocities, separation_distance):
    ''' Obstacle avoidance force
    '''
    # Get valid neighbors
    distances = np.linalg.norm(obstacles[idx] - obstacles, axis=1)
    j = np.where( (distances < separation_distance) & (distances>0))[0]
    if j.size == 0: return np.zeros(3)

    # obstacle avoidance
    sub =  obstacles[j] - obstacles[idx]
    norm = np.linalg.norm(obstacles[j], axis=1)[:,np.newaxis]

    div = np.divide(sub,norm)
    com = (div)*500
    o_i = np.sum(com, axis=0)

    return o_i


@wp.func
def compute_force(p: wp.vec3, 
                  v: wp.vec3,
                pn: wp.array(dtype=wp.vec3), 
                vn: wp.array(dtype=wp.vec3), 
                s_d: float, 
                c_d: float,
                a_d: float,
                e_d: float,
                dt: float,
                l_i: float,
                grid: wp.uint64,
                params: Params,
                ):

    s_i = separation(p, pn, s_d, grid)
    k_i = cohesion(p, pn, c_d, grid)
    m_i = alignment(p, vn, a_d, grid)

    S,K,M,X = params.S, params.K, params.M, params.X

    x_i = eccentricity(p, pn, e_d, grid)
    l_i = leadership(v, k_i, x_i, l_i, X, dt)
 
    # # o_i = obstacle(idx, obstacles, v, 5.0)

    # If a leader, apply the leadership force
    if l_i > 0.0:
        f = (s_i * S) + (k_i * K) + (m_i * M) #+ o_i
        f += f * (1.0+l_i) # Leader moves faster
    else:
        f = (s_i * S) + (k_i * K) + (m_i * M) #+ o_i

    return f, l_i
