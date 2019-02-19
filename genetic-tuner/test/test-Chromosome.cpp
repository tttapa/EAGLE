#include <gtest/gtest.h>
#include <Chromosome.hpp>

TEST(Chromosome, initializationAndRetrieve) {
    Chromosome<6> c = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(c[i], (double)i);
}

TEST(Chromosome, initializationAndRetrieveConst) {
    const Chromosome<6> c = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(c[i], (double)i);
}

TEST(Chromosome, write) {
    Chromosome<6> c = {42.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    c[0] = 0.0;
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(c[i], (double)i);
}

TEST(Chromosome, crossOver) {
    Chromosome<6> c1 = {10.0, 11.0, 12.0, 13.0, 14.0, 15.0};
    Chromosome<6> c2 = {20.0, 21.0, 22.0, 23.0, 24.0, 25.0};
    Chromosome<6> sum = {30.0, 32.0, 34.0, 36.0, 38.0, 40.0};
    Chromosome<6> child1;
    Chromosome<6> child2;
    crossOver(c1, c2, child1, child2);
    for (int i = 0; i < 6; i++)
        EXPECT_EQ(child1[i] + child2[i], sum[i]);

}