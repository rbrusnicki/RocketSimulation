function [Act_b] = Act_Plant(TVA_b_1, TVA_b_2, TVA_cmd_b_1, TVA_cmd_b_2)

Act_b = 0.9965 * TVA_b_1 - 0.3422 * TVA_b_2 + 0.2038 * TVA_cmd_b_1 + 0.142 * TVA_cmd_b_2;