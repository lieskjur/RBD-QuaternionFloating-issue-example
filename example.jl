using RigidBodyDynamics
using MeshCatMechanisms, Blink
using StaticArrays
using LinearAlgebra

z_axis = SVector(0.,0.,1.);

a = 2.
ρ = 1.

CubeMass(a,ρ) = ρ*a^3
CubeInertia(a,ρ) = ρ*a^5/6*I(3)

world = RigidBody{Float64}("world")
mechanism = Mechanism(world; gravity = -9.81*z_axis)

floatingJt = Joint("floatingJt",QuaternionFloating{Float64}())
cube_inertia = SpatialInertia( frame_after(floatingJt),
	com = z_axis*a/2,
	# com = zero(SVector{3}),
	moment = CubeInertia(a,ρ),
    mass = CubeMass(a,ρ)
    )
cube = RigidBody("cube",cube_inertia)
attach!(mechanism,world,cube,floatingJt)

# Mechanism State
state = MechanismState(mechanism)

# initial positions and velocities
set_configuration!(state,floatingJt,[1,0,0,0,0,0,5])
set_velocity!(state,floatingJt,zeros(6))

# initialization of variables for FD and ID
result = DynamicsResult{Float64}(mechanism);
nv = num_velocities(mechanism)
v̇ = SegmentedVector(Vector{Float64}(undef,nv), tree_joints(mechanism), num_velocities)

# ID solving and FD result verification
v̇ .= [1,1,1,1,1,1]
τ = inverse_dynamics(state,v̇)
dynamics!(result, state, τ)
print(result.v̇ ≈ v̇)