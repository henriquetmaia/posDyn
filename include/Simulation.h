// -----------------------------------------------------------------------------
// libDDG -- Simulation.h
// -----------------------------------------------------------------------------
//
// Position Based control of mesh interactions and dynamics
// see [Position Based Constraints - mueller et al 2007] for details
//

#ifndef DDG_SIMULATION_H
#define DDG_SIMULATION_H

#include "Face.h"
#include "HalfEdge.h"
#include "Vector.h"
#include "Mesh.h"
#include "Constraint.h"

using namespace std;

namespace DDG
{

   class Simulation
   {
      public:

         Simulation( vector< Mesh* > surface_ptrs, const double& numIters );

         ~Simulation( void );

         void step( const double& dt );

         void dampVelocities( void );

         void generateCollisionConstraints( void );

         void projectConstraints( void );

         void velocityUpdate( void );

         unsigned numExternalConstraints( void ){
            return m_externalConstraints.size();
         }

         void addExternalConstraint( Constraint* extConstraint ){
            m_externalConstraints.push_back( extConstraint );
         }

      protected:       

      private:

         unsigned m_solverIterations;

         vector< Mesh* > m_meshes;
         vector< Constraint* > m_externalConstraints;
         vector< Constraint* > m_collisionConstraints;
   };
}

// should we combine all the vertices of all objects into one vector_list and use indices to determine constraints
// or should I separate by meshes and then iterate over the meshes?

#endif

