#include <iostream>

#include <Chromosome.hpp>

using namespace std;

int main(int argc, const char *argv[]) {
    (void) argc;
    (void) argv;

    // Cross over two parents
    const Chromosome<6> parent1 = {10.0, 11.0, 12.0, 13.0, 14.0, 15.0};
    const Chromosome<6> parent2 = {20.0, 21.0, 22.0, 23.0, 24.0, 25.0};
    Chromosome<6> child1, child2;
    Chromosome<6> child3, child4;
    crossOver(parent1, parent2, child1, child2);
    crossOver(parent1, parent2, child3, child4);

    // Print results
    cout << "parent1: [";
    for (int i = 0; i < 6; i++)
        cout << parent1[i] << ',';
    cout << "]\r\n";

    cout << "parent2: [";
    for (int i = 0; i < 6; i++)
        cout << parent2[i] << ',';
    cout << "]\r\n";
        
    cout << "child1: [";
    for (int i = 0; i < 6; i++)
        cout << child1[i] << ',';
    cout << "]\r\n";
        
    cout << "child2: [";
    for (int i = 0; i < 6; i++)
        cout << child2[i] << ',';
    cout << "]\r\n";

    // Print children with toString()
    cout << "child3: " << toString(child3) << "\r\n";
    cout << "child4: " << toString(child4) << "\r\n";

    // Mutate children with factor 0.1
    mutate(child1, 0.1);
    mutate(child2, 0.1);
    mutate(child3, 0.1);
    mutate(child4, 0.1);

    // Print children with toString()
    cout << "child1: " << toString(child1) << "\r\n";
    cout << "child2: " << toString(child2) << "\r\n";
    cout << "child3: " << toString(child3) << "\r\n";
    cout << "child4: " << toString(child4) << "\r\n";


    return EXIT_SUCCESS;
}