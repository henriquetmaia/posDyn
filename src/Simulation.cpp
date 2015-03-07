
#include "Vector.h"
#include <Math.h> /* pi */
#include <algorithm>
#include <stdlib.h>
#include <eigen3/Eigen/Core>

#define EPSILON 1e-6

// TODO: make a macro FOR_ALL_VERTICES (?)

using namespace std;
namespace DDG
{

	Simulation :: Simulation( std::vector< Mesh* > dynamic_objects, const double& numIters )
	: m_solverIterations( numIters )
	{

	}

	void Simulation :: step( const double& dt )
	{

		// [ Mueller - 2007 (5) ]
		// TODO: these could be done in parallel as separate threads or GPU
		// apply external forces to the velocities of all the mesh's vertices
		for( unsigned ecId = 0; ecId < m_externalConstraints.size(); ++ecId ){
			for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){
			    for( VertexIter v = m_meshes[mIdx].vertices.begin(); v != m_meshes[mIdx].vertices.end(); ++v ){
			    	v.velocity = v.velocity + ( dt * v.invMass * m_externalConstraints[ecId].forceFunction( v.position ) );
			    }
			}
		}
		// [ Mueller - 2007 (6) ]
		dampVelocities();

		// [ Mueller - 2007 (7) ]
		// unconstrained position estimates		
		for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){
		    for( VertexIter v = m_meshes[mIdx].vertices.begin(); v != m_meshes[mIdx].vertices.end(); ++v ){
		    	v.estimate = v.position + ( dt * v.velocity );
		    }
		}

		// [ Mueller - 2007 (8) ]
		generateCollisionConstraints();

		// [ Mueller - 2007 (9)-(11) ]
		unsigned iters = 0;
		while( iters < m_solverIterations ){
			++iters;
			projectConstraints();
			// TODO: should we do a check to see if all constraints satisfied before iterations solverIterations completed?
		}

		// [ Mueller - 2007 (12)-(15) ]
		// unconstrained position estimates		
		for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){
		    for( VertexIter v = m_meshes[mIdx].vertices.begin(); v != m_meshes[mIdx].vertices.end(); ++v ){
		    	v.velocity = ( v.estimate - v.position ) / dt;
		    	v.position = v.estimate;
		    }
		}		

		// [ Mueller - 2007 (16) ]
		velocityUpdate();

	}

	void Simulation :: dampVelocities( void )
	{
		// [ Mueller - 2007 Sec: 3.5 ]
		for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){

			Vector x_cm;
			Vector v_cm;
			double sumMass = 0.;
		    for( VertexIter v = m_meshes[mIdx].vertices.begin(); v != m_meshes[mIdx].vertices.end(); ++v ){
		    	sumMass += v.mass;
		    	x_cm += v.position * v.mass;
		    	v_cm += v.velocity * v.mass;
		    }
			x_cm /= sumMass;
			v_cm /= sumMass;

			Vector r;
			Vector L;
			Eigen::Matrix3d I;
		    for( VertexIter v = m_meshes[mIdx].vertices.begin(); v != m_meshes[mIdx].vertices.end(); ++v ){
		    	r = v.position - x_cm;
		    	L += cross( r, v.mass * v.velocity );
		    	Eigen::Matrix3d singleI = Eigen::Matrix3d::Zero();
		    	singleI << 0.,  r[2], -r[1],
		    			-r[2],    0.,  r[0],
		    			 r[2], -r[0],    0.;
		    	I += singleI * singleI.transpose() * v.mass;
			}
			Vector ang_vel_omega = I.inverse() * L;

		    for( VertexIter v = m_meshes[mIdx].vertices.begin(); v != m_meshes[mIdx].vertices.end(); ++v ){
		    	r = v.position - x_cm;
		    	Vector delta_v = v_cm + cross( ang_vel_omega, r ) - v.velocity;
		    	v.velocity = v.velocity + ( m_meshes[mIdx].dampingStiffness() * delta_v );
			}

		}
	}

	void Simulation :: generateCollisionConstraints( void )
	{
		m_collisionConstraints.clear();

		// should probably do some hashing scheme first as a simple broad-phase cull

		for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){
		    for( VertexIter v = m_meshes[mIdx].vertices.begin(); v != m_meshes[mIdx].vertices.end(); ++v ){
		    	// Test vertex's 'position' and 'estimate' for collisions against nearby vertices/triangles
		    }
		}

	}

	void Simulation :: projectConstraints( void )
	{
		// Should we return a boolean if all the constraints are satisfied?

		unsigned numTotalConstraints = m_collisionConstraints.size();
		for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){
			numTotalConstraints += m_meshes[mIdx].numConstraints();
		}
	}	

	void Simulation :: velocityUpdate( void )
	{
		// [ Mueller - 2007 Sec: 3.4 (second paragraph) ]
		// handle friction and restitution for collisions


	}

}
