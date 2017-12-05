
#include "localization.h"

Localization::Localization(ros::NodeHandle& nodeHandle, Feature& features)
    : features(features)
{
    // Inicia as probabilidades
    P_S = std::vector<double>(18, 1.0 / 18.0);
}


void Localization::update()
{
    std::vector<double> P_S_new(18, 0);

    // Calculate the  P(S1| A) = SUM P(S1|S0 A)P(S0)
    for (int i = 0; i < 18; i++)
    {
        for (int j = 0; j < 18; j++)
        {
            P_S_new[j] += P_action[i][j] * P_S[i];
        }
    }

    // Calculate P(S1| Z) = P(Z | S1 ) P(S1| A)
    for (int i = 0; i < 18; i++)
    {
        // Calculate the probability of a feature on node i
        P_S_new[i] *= getP_feature(i);
    }

    P_S = P_S_new;
    normalise();
    show();
}


void Localization::normalise()
{
    double sum;

    for (int i = 0; i < 18; i++)
    {
        sum += P_S[i];
    }

    if (sum > 1)
    {
        for (int i = 0; i < 18; i++)
        {
            P_S[i] /= sum;
        }
    }
    else
    {
        for (int i = 0; i < 18; i++)
        {
            P_S[i] += (1 - sum) / 18;
        }
    }
}


void Localization::show()
{
    std::cout << "\n\n\n\n\n\n\n\n";
    std::cout << "A: " << P_S[0] << std::endl;
    std::cout << "B: " << P_S[1] << std::endl;
    std::cout << "C: " << P_S[2] << std::endl;
    std::cout << "D: " << P_S[3] << std::endl;
    std::cout << "E: " << P_S[4] << std::endl;
    std::cout << "F: " << P_S[5] << std::endl;
    std::cout << "G: " << P_S[6] << std::endl;
    std::cout << "H: " << P_S[7] << std::endl;
    std::cout << "I: " << P_S[8] << std::endl;
    std::cout << "J: " << P_S[9] << std::endl;
    std::cout << "K: " << P_S[10] << std::endl;
    std::cout << "L: " << P_S[11] << std::endl;
    std::cout << "M: " << P_S[12] << std::endl;
    std::cout << "N: " << P_S[13] << std::endl;
    std::cout << "O: " << P_S[14] << std::endl;
    std::cout << "P: " << P_S[15] << std::endl;
    std::cout << "Q: " << P_S[16] << std::endl;
    std::cout << "R: " << P_S[17] << std::endl;
}


double Localization::getP_feature(int node)
{
    double P = 0.5;

    if (fabs(features.get_corridor_width() - corridor_width[node]))
    {
        P += 0.3;
    }

    int expected_corners = internal_corners[node] + external_corners[node];
    int number_of_corners = features.get_internal_corners_count() + features.get_external_corners_count();

    if (fabs(expected_corners - number_of_corners) == 2)
    {
        // Should have seen two corner but did not see any
        P -= 0.15;
    }
    else if (fabs(expected_corners - number_of_corners) == 1)
    {
        // Doesn't change the probability, maybe it's behind
    }
    else
    {
        // See all the corners in the node
        P += 0.15;
    }

    return P;
}
