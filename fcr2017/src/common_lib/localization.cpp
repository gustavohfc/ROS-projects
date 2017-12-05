
#include <std_msgs/Float64MultiArray.h>
#include "localization.h"

#define N_NODES 18

Localization::Localization(ros::NodeHandle& nodeHandle, Feature& features)
    : features(features)
{
    pub_probabilities = nodeHandle.advertise<std_msgs::Float64MultiArray>("probabilities", 1);

    // Inicia as probabilidades
    resetProbabilities();
}


 void Localization::resetProbabilities()
 {
    P_S = std::vector<double>(N_NODES, 1.0 / N_NODES);
 }


// Update the node probability array
void Localization::update()
{
    std::vector<double> P_S_new(N_NODES, 0);

    // Calculate the  P(S1| A) = SUM P(S1|S0 A)P(S0)
    for (int i = 0; i < N_NODES; i++)
    {
        for (int j = 0; j < N_NODES; j++)
        {
            P_S_new[j] += P_action[i][j] * P_S[i];
        }
    }

    // Calculate P(S1| Z) = P(Z | S1 ) P(S1| A)
    for (int i = 0; i < N_NODES; i++)
    {
        // Calculate the probability of a feature on node i
        P_S_new[i] *= getP_feature(i);
    }

    P_S = P_S_new;
    normalise();
    show();
    send_data();
}


// Normalise the probability array
void Localization::normalise()
{
    double sum;

    for (int i = 0; i < N_NODES; i++)
    {
        sum += P_S[i];
    }

    for (int i = 0; i < N_NODES; i++)
    {
        P_S[i] /= sum;
    }
}


// Print the current probability of each node
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


// Calculate the probability of each feature in the current node
double Localization::getP_feature(int node)
{
    double P = 0.5;

    if (fabs(features.get_corridor_width() - corridor_width[node]) < 0.4)
    {
        P += 0.5;
    }

    int expected_corners = internal_corners[node] + external_corners[node];
    int number_of_corners = features.get_internal_corners_count() + features.get_external_corners_count();

    if (fabs(expected_corners - number_of_corners) == 2)
    {
        // Should have seen two corner but did not see any
        P -= 0.25;
    }
    else if (fabs(expected_corners - number_of_corners) == 1)
    {
        // Doesn't change the probability, maybe it's behind
    }
    else
    {
        // See all the corners in the node
        P += 0.25;
    }

    return P;
}


// Send the probability array to the python script that displays and saves this data
void Localization::send_data()
{
    std_msgs::Float64MultiArray message;

    for (int i = 0; i < N_NODES; i++)
    {
        message.data.push_back(P_S[i]);
    }

    pub_probabilities.publish(message);
}