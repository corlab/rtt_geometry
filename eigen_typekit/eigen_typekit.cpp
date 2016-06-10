// Copyright  (C)  2008  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "eigen_typekit.hpp"
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/types/Operators.hpp>
#include <rtt/types/OperatorTypes.hpp>
#include <rtt/types/TemplateConstructor.hpp>
#include <rtt/types/Types.hpp>
#include <rtt/Logger.hpp>
#include <rtt/internal/DataSources.hpp>
#include <rtt/internal/mystd.hpp>
#include <rtt/os/StartStopManager.hpp>
#include <rtt/types/TypekitRepository.hpp>
#include <rtt/internal/FusedFunctorDataSource.hpp>
#include <rtt/internal/DataSourceGenerator.hpp>
#include <boost/lexical_cast.hpp>


template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< Eigen::VectorXf >;
template class RTT_EXPORT RTT::internal::DataSource< Eigen::VectorXf >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< Eigen::VectorXf >;
template class RTT_EXPORT RTT::internal::AssignCommand< Eigen::VectorXf >;
template class RTT_EXPORT RTT::internal::ValueDataSource< Eigen::VectorXf >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< Eigen::VectorXf >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< Eigen::VectorXf >;
template class RTT_EXPORT RTT::OutputPort< Eigen::VectorXf >;
template class RTT_EXPORT RTT::InputPort< Eigen::VectorXf >;
template class RTT_EXPORT RTT::Property< Eigen::VectorXf >;
template class RTT_EXPORT RTT::Attribute< Eigen::VectorXf >;
template class RTT_EXPORT RTT::Constant< Eigen::VectorXf >;

template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::internal::DataSource< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::internal::AssignCommand< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::internal::ValueDataSource< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::OutputPort< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::InputPort< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::Property< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::Attribute< Eigen::MatrixXf >;
template class RTT_EXPORT RTT::Constant< Eigen::MatrixXf >;


#include <Eigen/Core>
namespace Eigen{

    using namespace RTT;
    using namespace RTT::detail;
    using namespace RTT::types;

    std::istream& operator>>(std::istream &is,MatrixXf& v){
      return is;
    }
    std::istream& operator>>(std::istream &is,VectorXf& v){
      return is;
    }

    float& get_item(VectorXf& v, int index)
    {
        if (index >= (int) (v.size()) || index < 0)
            return RTT::internal::NA<float&>::na();
        return v[index];
    }

    float get_item_copy(const VectorXf& v, int index)
    {
        if (index >= (int) (v.size()) || index < 0)
            return RTT::internal::NA<float>::na();
        return v[index];
    }


    int get_size(const VectorXf& v)
    {
        return v.size();
    }

    struct VectorTypeInfo : public types::TemplateTypeInfo<VectorXf,true>
#if (RTT_VERSION_MAJOR*100+RTT_VERSION_MINOR) >= 206
                          , public MemberFactory
#endif
    {
        VectorTypeInfo():TemplateTypeInfo<VectorXf, true >("eigen_vector")
        {
        };
        
#if (RTT_VERSION_MAJOR*100+RTT_VERSION_MINOR) >= 206
        bool installTypeInfoObject(TypeInfo* ti) {
            // aquire a shared reference to the this object
            boost::shared_ptr< VectorTypeInfo > mthis = boost::dynamic_pointer_cast<VectorTypeInfo >( this->getSharedPtr() );
            assert(mthis);
            // Allow base to install first
            TemplateTypeInfo<VectorXf,true>::installTypeInfoObject(ti);
            // Install the factories for primitive types
            ti->setMemberFactory( mthis );
            // Don't delete us, we're memory-managed.
            return false;
        }
#endif

        bool resize(base::DataSourceBase::shared_ptr arg, int size) const
        {
            if (arg->isAssignable()) {
	        RTT::internal::AssignableDataSource<VectorXf>::shared_ptr asarg = RTT::internal::AssignableDataSource<VectorXf>::narrow( arg.get() );
                asarg->set().resize( size );
                asarg->updated();
                return true;
            }
            return false;
        }

        virtual std::vector<std::string> getMemberNames() const {
            // only discover the parts of this struct:
            std::vector<std::string> result;
            result.push_back("size");
            result.push_back("capacity");
            return result;
        }

        base::DataSourceBase::shared_ptr getMember(base::DataSourceBase::shared_ptr item, const std::string& name) const {
            // the only thing we do is to check for an integer in name, otherwise, assume a part (size) is accessed:
            try {
                unsigned int indx = boost::lexical_cast<unsigned int>(name);
                // @todo could also return a direct reference to item indx using another DS type that respects updated().
                return getMember( item, new RTT::internal::ConstantDataSource<int>(indx));
            } catch(...) {}

            return getMember( item, new RTT::internal::ConstantDataSource<std::string>(name) );
        }

        base::DataSourceBase::shared_ptr getMember(base::DataSourceBase::shared_ptr item,
                                                   base::DataSourceBase::shared_ptr id) const {
            // discover if user gave us a part name or index:
            RTT::internal::DataSource<int>::shared_ptr id_indx = RTT::internal::DataSource<int>::narrow( RTT::internal::DataSourceTypeInfo<int>::getTypeInfo()->convert(id).get() );
            RTT::internal::DataSource<std::string>::shared_ptr id_name = RTT::internal::DataSource<std::string>::narrow( id.get() );
            if ( id_name ) {
                if ( id_name->get() == "size" || id_name->get() == "capacity") {
                    try {
                        return RTT::internal::newFunctorDataSource(&get_size, RTT::internal::GenerateDataSource()(item.get()) );
                    } catch(...) {}
                }
            }

            if ( id_indx ) {
                try {
                    if ( item->isAssignable() )
                        return RTT::internal::newFunctorDataSource(&get_item, RTT::internal::GenerateDataSource()(item.get(), id_indx.get() ) );
                    else
                        return RTT::internal::newFunctorDataSource(&get_item_copy, RTT::internal::GenerateDataSource()(item.get(), id_indx.get() ) );
                } catch(...) {}
            }
            if (id_name) {
                log(Error) << "EigenVectorTypeInfo: No such member : " << id_name->get() << endlog();
            }
            if (id_indx) {
                log(Error) << "EigenVectorTypeInfo: Invalid index : " << id_indx->get() <<":"<< id_indx->getTypeName() << endlog();
            }
            if ( !id_name && ! id_indx)
                log(Error) << "EigenVectorTypeInfo: Not a member or index : " << id <<":"<< id->getTypeName() << endlog();
            return base::DataSourceBase::shared_ptr();
        }


        virtual bool decomposeTypeImpl(const VectorXf& vec, PropertyBag& targetbag) const
        {
            targetbag.setType("eigen_vector");
            int dimension = vec.rows();
            std::string str;

            if(!targetbag.empty())
                return false;

            for ( int i=0; i < dimension ; i++){
                std::stringstream out;
                out << i+1;
                str = out.str();
                targetbag.add( new Property<float>(str, str +"th element of vector",vec(i)) ); // Put variables in the bag
            }

            return true;
        };

      virtual bool composeTypeImpl(const PropertyBag& bag, VectorXf& result) const{

            if ( bag.getType() == "eigen_vector" ) {
                int dimension = bag.size();
                result.resize( dimension );

                // Get values
                for (int i = 0; i < dimension ; i++) {
                    std::stringstream out;
                    out << i+1;
                    Property<float> elem = bag.getProperty(out.str());
                    if(elem.ready())
                        result(i) = elem.get();
                    else{
                        log(Error)<<"Could not read element "<<i+1<<endlog();
                        return false;
                    }
                }
            }else{
                log(Error) << "Composing Property< VectorXf > :"
                           << " type mismatch, got type '"<< bag.getType()
                           << "', expected type "<<"eigen_vector."<<endlog();
                return false;
            }
            return true;
        };
    };

    struct MatrixTypeInfo : public types::TemplateTypeInfo<MatrixXf,true>{
        MatrixTypeInfo():TemplateTypeInfo<MatrixXf, true >("eigen_matrix"){
        };


        bool decomposeTypeImpl(const MatrixXf& mat, PropertyBag& targetbag) const{
            targetbag.setType("eigen_matrix");
            unsigned int dimension = mat.rows();
            if(!targetbag.empty())
                return false;

            for ( unsigned int i=0; i < dimension ; i++){
                std::stringstream out;
                out << i+1;
                targetbag.add( new Property<VectorXf >(out.str(), out.str() +"th row of matrix",mat.row(i) )); // Put variables in the bag
            }

            return true;
        };

        bool composeTypeImpl(const PropertyBag& bag, MatrixXf& result) const{
            if ( bag.getType() == "eigen_matrix" ) {
                unsigned int rows = bag.size();
                unsigned int cols = 0;
                // Get values
                for (unsigned int i = 0; i < rows ; i++) {
                    std::stringstream out;
                    out << i+1;
                    Property<PropertyBag> row_bag =  bag.getProperty(out.str());
                    if(!row_bag.ready()){
                        log(Error)<<"Could not read row "<<i+1<<endlog();
                        return false;
                    }
                    Property<VectorXf > row_p(row_bag.getName(),row_bag.getDescription());
                    if(!(row_p.compose(row_bag))){
                        log(Error)<<"Could not compose row "<<i+1<<endlog();
                        return false;
                    }
                    if(row_p.ready()){
                        if(i==0){
                            cols = row_p.get().size();
                            result.resize(rows,cols);
                        } else
                            if(row_p.get().rows()!=(int)cols){
                                log(Error)<<"Row "<<i+1<<" size does not match matrix columns"<<endlog();
                                return false;
                            }
                        result.row(i)=row_p.get();
                    }else{
                        log(Error)<<"Property of Row "<<i+1<<"was not ready for use"<<endlog();
                        return false;
                    }
                }
            }else {
                log(Error) << "Composing Property< MatrixXf > :"
                           << " type mismatch, got type '"<< bag.getType()
                           << "', expected type "<<"ublas_matrix."<<endlog();
                return false;
            }
            return true;
        };
    };

    struct vector_index
        : public std::binary_function<const VectorXf&, int, float>
    {
        float operator()(const VectorXf& v, int index) const
        {
            if ( index >= (int)(v.size()) || index < 0)
                return 0.0;
            return v(index);
        }
    };

    struct get_size
        : public std::unary_function<const VectorXf&, int>
    {
        int operator()(const VectorXf& cont ) const
        {
            return cont.rows();
        }
    };

    struct vector_index_value_constructor
        : public std::binary_function<int,float,const VectorXf&>
    {
        typedef const VectorXf& (Signature)( int, float );
        mutable boost::shared_ptr< VectorXf > ptr;
        vector_index_value_constructor() :
            ptr( new VectorXf ){}
        const VectorXf& operator()(int size,float value ) const
        {
            ptr->resize(size);
            (*ptr)=Eigen::VectorXf::Constant(size,value);
            return *(ptr);
        }
    };

    struct vector_index_array_constructor
        : public std::unary_function<std::vector<float >,const VectorXf&>
    {
        typedef const VectorXf& (Signature)( std::vector<float > );
        mutable boost::shared_ptr< VectorXf > ptr;
        vector_index_array_constructor() :
            ptr( new VectorXf ){}
        const VectorXf& operator()(std::vector<float > values) const
        {
            (*ptr)=VectorXf::Map(&values[0],values.size());
            return *(ptr);
        }
    };

    struct vector_index_constructor
        : public std::unary_function<int,const VectorXf&>
    {
        typedef const VectorXf& (Signature)( int );
        mutable boost::shared_ptr< VectorXf > ptr;
        vector_index_constructor() :
            ptr( new VectorXf() ){}
        const VectorXf& operator()(int size ) const
        {
            ptr->resize(size);
            return *(ptr);
        }
    };

    struct matrix_index
        : public std::ternary_function<const MatrixXf&, int, int, float>
    {
        float operator()(const MatrixXf& m, int i, int j) const{
            if ( i >= (int)(m.rows()) || i < 0 || j<0 || j>= (int)(m.cols()))
                return 0.0;
            return m(i,j);
        }
    };

    struct matrix_i_j_constructor
        : public std::binary_function<int,int,const MatrixXf&>
    {
        typedef const MatrixXf& (Signature)( int, int );
        mutable boost::shared_ptr< MatrixXf > ptr;
        matrix_i_j_constructor() :
            ptr( new MatrixXf() ){}
        const MatrixXf& operator()(int size1,int size2) const
        {
            ptr->resize(size1,size2);
            return *(ptr);
        }
    };


    std::string EigenTypekitPlugin::getName()
    {
        return "Eigen";
    }

    bool EigenTypekitPlugin::loadTypes()
    {
        RTT::types::TypeInfoRepository::Instance()->addType( new VectorTypeInfo() );
        RTT::types::TypeInfoRepository::Instance()->addType( new MatrixTypeInfo() );
        return true;
    }

    bool EigenTypekitPlugin::loadConstructors()
    {
        RTT::types::Types()->type("eigen_vector")->addConstructor(types::newConstructor(vector_index_constructor()));
        RTT::types::Types()->type("eigen_vector")->addConstructor(types::newConstructor(vector_index_value_constructor()));
        RTT::types::Types()->type("eigen_vector")->addConstructor(types::newConstructor(vector_index_array_constructor()));
        RTT::types::Types()->type("eigen_matrix")->addConstructor(types::newConstructor(matrix_i_j_constructor()));
        return true;
    }

    bool EigenTypekitPlugin::loadOperators()
    {
        RTT::types::OperatorRepository::Instance()->add( newBinaryOperator( "[]", vector_index() ) );
        //RTT::types::OperatorRepository::Instance()->add( newDotOperator( "size", get_size() ) );
        //RTT::types::OperatorRepository::Instance()->add( newTernaryOperator( "[,]", matrix_index() ) );
        return true;
    }
}

ORO_TYPEKIT_PLUGIN(Eigen::EigenTypekitPlugin)
