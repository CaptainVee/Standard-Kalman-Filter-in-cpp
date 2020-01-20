# Standard-Kalman-Filter-in-cpp
Kalman Filter Implementation in C++

## Notes for using the Eigen Library:
- You can create a vertical vector of two elements with a command like this:
```sh
VectorXd my_vector(2);
```
- You can use the so called comma initializer to set all the coefficients to some values:
```sh
my_vector << 10, 20;
```
- and you can use the cout command to print out the vector:
```sh
cout << my_vector << endl;
```
- The matrices can be created in the same way. For example, This is an initialization of a 2 by 2 matrix with the values 1, 2, 3, and 4:
```sh
MatrixXd my_matrix(2,2);
my_matrix << 1, 2,
             3, 4;
```           
- You can use the same comma initializer or you can set each matrix value explicitly. For example, that's how we can change the matrix elements in the second row:
```sh
my_matrix(1,0) = 11;    //second row, first column
my_matrix(1,1) = 12;    //second row, second column
```
- Also, you can compute the transpose of a matrix with the following command:
```sh
MatrixXd my_matrix_t = my_matrix.transpose();
```
- And here is how you can get the matrix inverse:
```sh
MatrixXd my_matrix_i = my_matrix.inverse();
```
- For multiplying the matrix m with the vector b you can write this in one line as let’s say matrix c equals m times v:
```sh
MatrixXd another_matrix;
another_matrix = my_matrix*my_vector;
```
