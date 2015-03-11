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

         Simulation( void );

         ~Simulation( void );

         void step( const double& dt );

         void dampVelocities( void );

         void generateCollisionConstraints( void );

         void projectConstraints( void );

         void velocityUpdate( void );

         void setNumIters( const unsigned& iterations );

         unsigned numExternalConstraints( void ){
            return m_externalConstraints.size();
         }

         void addExternalConstraint( Constraint* extConstraint ){
            m_externalConstraints.push_back( extConstraint );
         }

         int addMesh( const std::string& filename );

         Mesh* operator[] ( const unsigned& index );
         // returns reference to the specified mesh (0-based indexing)

         const Mesh* operator[] ( const unsigned& index ) const;
         // returns const reference to the specified mesh (0-based indexing)         

         const unsigned size( void );
         // returns number of meshes contained in sim

      protected:       

      private:

         vector< Mesh* > m_meshes;

         unsigned m_solverIterations;

         vector< Constraint* > m_externalConstraints;
         vector< Constraint* > m_collisionConstraints;
   };
}

// should we combine all the vertices of all objects into one vector_list and use indices to determine constraints
// or should I separate by meshes and then iterate over the meshes?

#endif

