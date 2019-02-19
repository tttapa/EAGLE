#include <Chromosome.hpp>
#include <gtest/gtest.h>

TEST(Chromosome, initializationAndRetrieve) {
    // Chromosome<6> c = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    Chromosome<6> c = {0, 1, 2, 3, 4, 5};
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(c[i], i);
}

TEST(Chromosome, initializationAndRetrieveConst) {
    const Chromosome<6> c = {0, 1, 2, 3, 4, 5};
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(c[i], i);
}

TEST(Chromosome, write) {
    Chromosome<6> c = {42, 1, 2, 3, 4, 5};
    c[0]            = 0;
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(c[i], i);
}

TEST(Chromosome, crossOver) {
    Chromosome<6> parent1 = {10, 11, 12, 13, 14, 15};
    Chromosome<6> parent2 = {20, 21, 22, 23, 24, 25};
    // Chromosome<6> sum = {30.0, 32.0, 34.0, 36.0, 38.0, 40.0};
    Chromosome<6> child1;
    Chromosome<6> child2;
    crossOver(parent1, parent2, child1, child2);
    // for (int i = 0; i < 6; i++)
    //     EXPECT_EQ(child1[i] + child2[i], sum[i]);
    EXPECT_EQ(child1 + child2, parent1 + parent2);
}