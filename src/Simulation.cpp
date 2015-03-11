
#include "Vector.h"
#include <Math.h> /* pi */
#include <algorithm>
#include <stdlib.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "Simulation.h"

// TODO: make a macro FOR_ALL_VERTICES (?)

using namespace std;
namespace DDG
{

	Simulation :: Simulation( void )
	{
		m_solverIterations = 10;
	}


	Simulation :: ~Simulation( void )
	{}

	void Simulation :: setNumIters( const unsigned& iterations ){
		m_solverIterations = iterations;
	}

	Mesh* Simulation :: operator[]( const unsigned& index )
	{
		return ( m_meshes )[ index ];
	}

	const Mesh* Simulation :: operator[]( const unsigned& index ) const
	{
		return ( m_meshes )[ index ];
	}

	const unsigned Simulation :: size( void ){
		return m_meshes.size();
	}

	void Simulation :: step( const double& dt )
	{

		// [ Mueller - 2007 (5) ]
		// TODO: these could be done in parallel as separate threads or GPU
		// apply external forces to the velocities of all the mesh's vertices
		for( unsigned ecId = 0; ecId < m_externalConstraints.size(); ++ecId ){
			for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){
			    for( VertexIter v = m_meshes[mIdx]->vertices.begin(); v != m_meshes[mIdx]->vertices.end(); ++v ){
			    	v->velocity = v->velocity + ( dt * v->invMass * m_externalConstraints[ecId]->forceFunction( v->position ) );
			    }
			}
		}
		// [ Mueller - 2007 (6) ]
		// dampVelocities();

		// [ Mueller - 2007 (7) ]
		// unconstrained position estimates	from external constraints
		for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){
		    for( VertexIter v = m_meshes[mIdx]->vertices.begin(); v != m_meshes[mIdx]->vertices.end(); ++v ){
		    	v->estimate = v->position + ( dt * v->velocity );
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
		    for( VertexIter v = m_meshes[mIdx]->vertices.begin(); v != m_meshes[mIdx]->vertices.end(); ++v ){
		    	// v->velocity = ( v->estimate - v->position ) / dt;
		    	v->position = v->estimate;
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
		    for( VertexIter v = m_meshes[mIdx]->vertices.begin(); v != m_meshes[mIdx]->vertices.end(); ++v ){
		    	sumMass += v->mass;
		    	x_cm += v->position * v->mass;
		    	v_cm += v->velocity * v->mass;
		    }
			x_cm /= sumMass;
			v_cm /= sumMass;

			Vector r;
			Vector L;
			Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
		    for( VertexIter v = m_meshes[mIdx]->vertices.begin(); v != m_meshes[mIdx]->vertices.end(); ++v ){
		    	r = v->position - x_cm;
		    	L += cross( r, v->mass * v->velocity );
		    	Eigen::Matrix3d singleI = Eigen::Matrix3d::Zero();
		    	singleI << 0.,  r[2], -r[1],
		    			-r[2],    0.,  r[0],
		    			 r[2], -r[0],    0.;
		    	I += singleI * singleI.transpose() * v->mass;
			}
			Eigen::Vector3d L_tmp ( L[0], L[1], L[2] );
			Eigen::Vector3d omegaTemp = I.inverse() * L_tmp;
			Vector ang_vel_omega( omegaTemp[0], omegaTemp[1], omegaTemp[2] );

		    for( VertexIter v = m_meshes[mIdx]->vertices.begin(); v != m_meshes[mIdx]->vertices.end(); ++v ){
		    	r = v->position - x_cm;
		    	Vector delta_v = v_cm + cross( ang_vel_omega, r ) - v->velocity;
		    	v->velocity = v->velocity + ( m_meshes[mIdx]->dampingStiffness() * delta_v );
			}

		}
	}

	void Simulation :: generateCollisionConstraints( void )
	{
		// TODO:
		m_collisionConstraints.clear();

		// should probably do some hashing scheme first as a simple broad-phase cull

		for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){
		    for( VertexIter v = m_meshes[mIdx]->vertices.begin(); v != m_meshes[mIdx]->vertices.end(); ++v ){
		    	// Test vertex's 'position' and 'estimate' for collisions against nearby vertices/triangles
		    }
		}
	}

	void Simulation :: projectConstraints( void )
	{
		// Should we return a boolean if all the constraints are satisfied?
		// TODO:
		unsigned numCollisions = m_collisionConstraints.size();
		unsigned numTotalConstraints = numCollisions;
		for( unsigned mIdx = 0; mIdx < m_meshes.size(); ++mIdx ){
			for( unsigned cIdx = 0; cIdx < m_meshes[mIdx]->numConstraints(); ++cIdx ){
				// why not make a resolve constraints function?
				m_meshes[mIdx]->m_constraints[cIdx]->forceFunction();
			}
			numTotalConstraints += m_meshes[mIdx]->numConstraints();
		}

		// TODO: do stuff with equality and inequality checks here
/*
		for( unsigned cIdx = 0; cIdx < fullConstraints.size(); ++cIdx ){
			// for the indices in the constraint:

			// compute delta_estimate
			v->estimate += -scaling * v->invMass * gradEstimate * fullConstraints[cIdx].forceFunction(  );

			// within constraints, if type inequality then only project if unsatisfied


		}
*/
	}	

	void Simulation :: velocityUpdate( void )
	{
		// [ Mueller - 2007 Sec: 3.4 (second paragraph) ]
		// handle friction and restitution for collisions

		// TODO:
		for( unsigned cIdx = 0; cIdx < m_collisionConstraints.size(); ++cIdx ){
			// dampen perpendicular motion and reflect off bounce direction...
		}
	}
	static bool isFirst = true;
	int Simulation :: addMesh( const std::string& filename ){
		Mesh* object = new Mesh();
		if( object->read( filename ) ){
			return 1; // error
		} 

		Vector shiftPos( 0., 0., 0. );
		Vector initialVel( 0., 0., 0. );
		double vertexMasses = 1.0;
		double constraintStiffness = 0.7;
		bool isFixed;
		if( isFirst ){
			isFixed = false;
			isFirst = false;
		}
		else{
			isFixed = true;
		}
		// bool isFixed = false;

		object->initDynamics( shiftPos, initialVel, vertexMasses, constraintStiffness, isFixed );

		m_meshes.push_back( object );
		return 0; // success
	}

}
