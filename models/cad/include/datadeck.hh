#ifndef __DATADECK_HH__
#define __DATADECK_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (DATADECK class)
LIBRARY DEPENDENCY:
      ((../src/datadeck.cpp))
*******************************************************************************/
#include <boost/serialization/vector.hpp>

#include <string>
#include <vector>
#include <iostream>

using namespace std;

// sizing of arrays
#define CHARN 50   ///< character numbers in variable names
#define CHARL 220  ///< character numbers in a line

class Table
{
    private:
        string name;
        int dim;
        int var1_dim;
        int var2_dim;
        int var3_dim;
    public:
        std::vector<double> var1_values;
        std::vector<double> var2_values;
        std::vector<double> var3_values;
        std::vector<double> data;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            ar & name;
            ar & dim;
            ar & var1_dim;
            ar & var2_dim;
            ar & var3_dim;

            ar & var1_values;
            ar & var2_values;
            ar & var3_values;
            ar & data;
        }

        Table(){}
        virtual ~Table()
        {
        }

        ///////////////////////////////////////////////////////////////////////////
        //Getting dimension of table
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        int get_dim(){return dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Getting name of table
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        string get_name(){return name;}

        ///////////////////////////////////////////////////////////////////////////
        //Getting 1. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        int get_var1_dim(){return var1_dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Getting 2. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        int get_var2_dim(){return var2_dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Getting 3. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        int get_var3_dim(){return var3_dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting dimension of table
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_dim(int table_dim){dim=table_dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting name of table
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_name(string tbl_name){name=tbl_name;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting 1. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var1_dim(int size){var1_dim=size;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting 2. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var2_dim(int size){var2_dim=size;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting 3. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var3_dim(int size){var3_dim=size;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting 1. independent variable values
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var1_value(int offset,double value){
            var1_values[offset]=value;
        }
        ///////////////////////////////////////////////////////////////////////////
        //Setting 2. independent variable values
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var2_value(int offset,double value){
            var2_values[offset]=value;
        }
        ///////////////////////////////////////////////////////////////////////////
        //Setting 3. independent variable values
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var3_value(int offset,double value){
            var3_values[offset]=value;
        }
        ///////////////////////////////////////////////////////////////////////////
        //Setting tablular data values
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_data(int offset,double value){
            data[offset]=value;
        }
};

class Datadeck
{
    private:
        string title;
        int capacity;
        int tbl_counter;
        std::vector<Table*> table_ptr;

    public:
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            ar & title;
            ar & capacity;
            ar & tbl_counter;
            ar & table_ptr;
        }

        Datadeck(){}
        Datadeck(char *file_name);
        virtual ~Datadeck(){}

        ///////////////////////////////////////////////////////////////////////////////
        //Allocating memory  table deck title
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void alloc_mem(){
            table_ptr = std::vector<Table*>(capacity);
            //for(int i = 0; i < capacity; i++)
                //table_ptr[i] = new Table();
        }

        ///////////////////////////////////////////////////////////////////////////////
        //Setting table deck title
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void set_title(string deck_title){title=deck_title;}

        ///////////////////////////////////////////////////////////////////////////////
        //Getting table deck title
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        string get_title(){return title;}

        ///////////////////////////////////////////////////////////////////////////////
        //Setting total number of tables
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void set_capacity(int table_numbers){capacity=table_numbers;}

        ///////////////////////////////////////////////////////////////////////////////
        //Getting total number of tables
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        int get_capacity(){return capacity;}

        ///////////////////////////////////////////////////////////////////////////////
        //Setting table counter
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void set_counter(int count){tbl_counter=count;}

        ///////////////////////////////////////////////////////////////////////////////
        //Getting table counter
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        int get_counter(){return tbl_counter;}

        ///////////////////////////////////////////////////////////////////////////////
        //Adding a table pointer to the table list
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void add_table(Table &pt)
        {
            if(tbl_counter<capacity){
                table_ptr[tbl_counter]=&pt;
            }
        }
        ///////////////////////////////////////////////////////////////////////////////
        //Overloaded operator [] returns a 'Table' pointer
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        Table * operator[](int slot)
        {
            if(slot>=0 && slot<capacity)
                return table_ptr[slot];
            else
            {
                cout<<"*** Bad pointer value of table deck: "<<slot<<'\n';
                return 0;
            }
        }

        ///////////////////////////////////////////////////////////////////////////////
        //Getting table pointer
        //030717 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        Table * get_tbl(int slot){return table_ptr[slot];}

        ///////////////////////////////////////////////////////////////////////////////
        //Single independent variable look-up
        //constant extrapolation at the upper end, slope extrapolation at the lower end
        //
        //030717 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double look_up(string name,double value1);

        ///////////////////////////////////////////////////////////////////////////////
        //Two independent variables look-up
        // Constant extrapolation at the upper end, slope extrapolation at the lower end
        //
        //030717 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double look_up(string name,double value1,double value2);

        ///////////////////////////////////////////////////////////////////////////////
        //Three independent variables look-up
        //constant extrapolation at the upper end, slope extrapolation at the lower end
        //
        //030723 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double look_up(string name,double value1,double value2,double value3);

        ///////////////////////////////////////////////////////////////////////////////
        //Table index finder
        //This is a binary search method it is O(lgN)
        //* Returns array locater of the discrete_variable just below variable
        //* Keeps max or min array locater if variable is outside those max or min
        //
        //010628 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        int find_index(int max,double value, std::vector<double> list);

        ///////////////////////////////////////////////////////////////////////////////
        //Linear one-dimensional interpolation
        // Constant extrapolation beyond max values of X1
        // Slope extrapolation beyond min values of X1
        //
        //030717 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double interpolate(int ind,int ind2,int slot,double val);

        ///////////////////////////////////////////////////////////////////////////////
        //Linear, two-dimensional interpolation
        // Constant extrapolation beyond max values of X1 and X2
        // Slope extrapolation beyond min values of X1 and X2
        //
        //030718 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double interpolate(int ind10,int ind11,int ind20,int ind21,int slot,double value1,
                            double value2);

        ///////////////////////////////////////////////////////////////////////////////
        //Linear, three-dimensional interpolation
        //Constant extrapolation beyond max values of X1, X2 and X3
        //Slope extrapolation beyond min values of X1, X2 and X3
        //
        //030723 Created by Peter Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double interpolate(int ind10,int ind11,int ind20,int ind21,int ind30,int ind31,
                                     int slot,double value1,double value2,double value3);

};

#endif  // __DATADECK_HH__
