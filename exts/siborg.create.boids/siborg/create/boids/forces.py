'''
Hartman, Christopher, and Bedrich Benes. “Autonomous Boids.” 
Computer Animation and Virtual Worlds, vol. 17, no. 3-4, 2006, pp. 199-206, doi:10.1002/cav.123.

Many things are not clear in the paper, so modifications may have been done. 
'''

import warp as wp
import numpy as np

def separation(idx, positions, velocities, separation_distance):
    ''' Separation force
    '''
    # Get valid neighbors
    distances = np.linalg.norm(positions[idx] - positions, axis=1)
    j = np.where( (distances < separation_distance) & (distances>0) )[0]
    if j.size == 0: return np.zeros(3)

    # Steerforce
    s_i = np.sum(positions[idx] - positions[j], axis=0)

    return s_i


def cohesion(idx, positions, velocities, cohesion_distance):

    # Get valid neighbors
    distances = np.linalg.norm(positions[idx] - positions, axis=1)
    j = np.where( (distances < cohesion_distance) & (distances>0))[0]
    if j.size == 0: return np.zeros(3)
    
    # Cohesion vector
    V_i = positions[j]
    c_i = np.sum(V_i/( (V_i.size/3)), axis=0)

    # Cohesion vector
    k_i = c_i - positions[idx]

    return k_i

def alignment(idx, positions, velocities, alignment_distance):

    # Get valid neighbors
    distances = np.linalg.norm(velocities[idx] - velocities, axis=1)
    j = np.where( (distances < alignment_distance) & (distances>0))[0]
    if j.size == 0: return np.zeros(3)
    
    # Alignment vector
    V_i = velocities[j]

    m_i = np.sum(V_i/( (V_i.size/3)))

    return m_i

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


def eccentricity(idx, positions, velocities, eccentricity_distance):
    
    # Distance boid can see
    e = eccentricity_distance

    distances = np.linalg.norm(positions[idx] - positions, axis=1)
    j = np.where((distances < eccentricity_distance) & (distances>0))[0]
    if j.size == 0: return 0
    
    V_i = positions[j] 

    # Center of density
    c_i = np.sum(V_i/( ( (V_i.size/3) ) ), axis=0)

    # Distance of boid to the center of density
    c_i = np.linalg.norm(positions[idx] - c_i, axis=0)

    # The divisor e is the visibility range of the boid, so the
    # value xi is normalized xi ∈ (0, 1)

    x_i = c_i/e

    return x_i


def leadership(idx, positions, velocities, k_i, x_i, leaders, X, dt):
    ''' 
    Not clear what the paper is explaining with the algorithm steps. Here is the guess:

    - We need to decide when the bird becomes a leader, and after that, 
    how long does it stay a leader. 
    - when timer is 0, the bird is not a leader
    - the timer stops birds from becoming leaders when they are leaders

    dt is timestep, currently 1 is used to start the leader countdown
    '''

    leader = leaders[idx]

    # If currently and still is a leader, apply the time-based leadership force
    if leader > 0:
        return leader - dt
        
    # Check where the bird is in relation to the flock
    v_i = np.linalg.norm(velocities[idx])
    k_i = np.linalg.norm(k_i)
    l_i = np.dot(k_i, v_i)

    # If the dot product is negative, the bird is behind the flock, and should not be be a leader
    if l_i < 0: 
        return -1

    # Trigger runaway
    mu = 0.5
    gauss_rand = np.random.normal(mu)
    if (x_i >= X*gauss_rand): 
        return 1
   
    return -1


def compute_forces(idx, p, v, leaders, obstacles, dt):

    X = 0.5

    separation_distance = 3.5
    alignment_distance = 5.5
    cohesion_distance = 25.0
    eccentricity_distance = 5.0

    min_speed = 5
    max_speed = 30.0

    # Reset velocity if it is too high
    if np.linalg.norm(v[idx]) > max_speed:
        # Clamp velocity
        v[idx] = v[idx] / np.linalg.norm(v[idx]) * max_speed
   
    s_i = separation(idx, p, v, separation_distance)
    k_i = cohesion(idx, p, v, cohesion_distance)
    m_i = alignment(idx, p, v, alignment_distance)
    x_i = eccentricity(idx, p, v, eccentricity_distance)
    l_i = leadership(idx, p, v, k_i, x_i, leaders, X, dt)

    # Obstacles (should include walls)
    # obstacles[idx] = p[idx] # Hack to make boid part of the obstacle list
    # o_i = obstacle(idx, obstacles, v, 5.0)
    o_i = np.zeros(3)

    return s_i, k_i, m_i, o_i, l_i


def set_forces(idx, s_i, k_i, m_i, o_i, l_i):

    S,K,M = 0.7, 0.95, 0.01

    # If a leader, apply the leadership force
    if l_i > 0:
        force = (s_i * S) + (k_i * K) + (m_i * M) #+ o_i
        force += force* (1.0+l_i) # Leader moves faster
    else:
        force = (s_i * S) + (k_i * K) + (m_i * M) #+ o_i


    # force = o_i
    # Clamp the force
    # force = np.clip(force, -100, 100)

    return force