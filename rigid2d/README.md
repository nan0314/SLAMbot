# Rigid 2D transformation Library
A library for handling transformations in SE(2).

# Conceptual Questions
1. What is the difference between a class and a struct in C++?

    * The visibility of members defaults (when unspecified) to public in structs and private in classes. 

2. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specic C++ core guidelines in your answer)?


    * According to C.2, we want Vector2D to be a struct because all of its members can vary independently. 
    * According to C.8, we want Transform2D to be a class becuase we have private members.

3. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

    * According to C.48, we want to use explicit on constructors with only one argument to avoid unintended conversions 
    (unless we actually want an implicit conversion)

4. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):

- Propose three different designs for implementing the ~normalize~ functionality
- Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
- Which of the methods would you implement and why?

    * Method 1: Create a constructor to calculate and store the normal of the vector
        * This method keeps all the related data together in the original struct format but does not insure that the 
        normalized data will be updated when the x and y are changed. If the user is not aware how the struct is set up, it is likely that this data will not remain accurate.
    * Method 2: Create an outside helper function that takes a Vector2D and outputs a normalized Vector2D.
        * This allows the user to access the normalized vector according to the current status of the input
        Vector2D, however it goes against the C++ core guidelines by separating related data (not storing it 
        in the struct).
    * Method 3: Create a public method within the Vector2D struct that returns a a normalized vector according to
        the current status of the vector.
        * This addreses the problems in Methods 1 and 2-- it insures that up to date data is returned at the time of 
        call and keeps related methods together in the struct. I would implement this method for these reasons.
   
5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not (Refer to C++ core guidelines in your answer)?

    * According to Con.2, A member function should be marked const unless it changes the object’s observable state. This gives a more precise statement of design intent, better readability, more errors caught by the compiler, and sometimes more optimization opportunities.