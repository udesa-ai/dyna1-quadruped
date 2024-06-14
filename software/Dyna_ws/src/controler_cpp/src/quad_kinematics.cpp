#include "quad_kinematics.hpp"

QuadModel::QuadModel(){}

QuadModel::QuadModel(float shoulder_length,
            float elbow_length,
            float wrist_length,
            float hip_x, 
            float hip_y, 
            float foot_x, 
            float foot_y,
            float height,
            float com_offset):
            shoulder_length(shoulder_length),
            elbow_length(elbow_length),
            wrist_length(wrist_length),
            hip_x(hip_x),
            hip_y(hip_y),
            foot_x(foot_x),
            foot_y(foot_y),
            height(height),
            com_offset(com_offset)
    {
        Legs.insert(std::pair<uint8_t, LegK>(FL, LegK( LEFT, shoulder_length,
                                             elbow_length, wrist_length)));

        Legs.insert(std::pair<uint8_t, LegK>(FR, LegK( RIGHT, shoulder_length,
                                             elbow_length, wrist_length)));

        Legs.insert(std::pair<uint8_t, LegK>(BL, LegK( LEFT, shoulder_length,
                                             elbow_length, wrist_length)));

        Legs.insert(std::pair<uint8_t, LegK>(BR, LegK( RIGHT, shoulder_length,
                                             elbow_length, wrist_length)));

        names[0] = FL;
        names[1] = FR;
        names[2] = BL;
        names[3] = BR;
        

        Eigen::Matrix3f Rwb = Eigen::Matrix3f::Identity();

        Eigen::Vector3f ph_FL {(float)hip_x/2, (float)hip_y/2, 0.0f};
        WorldToHip.insert(std::pair<uint8_t, Eigen::Matrix4f>(FL, RpToTrans(Rwb, ph_FL)));

        Eigen::Vector3f ph_FR {(float)hip_x/2, -(float)hip_y/2, 0.0f};
        WorldToHip.insert(std::pair<uint8_t, Eigen::Matrix4f>(FR, RpToTrans(Rwb, ph_FR)));

        Eigen::Vector3f ph_BL {-(float)hip_x/2, (float)hip_y/2, 0.0f};
        WorldToHip.insert(std::pair<uint8_t, Eigen::Matrix4f>(BL, RpToTrans(Rwb, ph_BL)));

        Eigen::Vector3f ph_BR {-(float)hip_x/2, -(float)hip_y/2, 0.0f};
        WorldToHip.insert(std::pair<uint8_t, Eigen::Matrix4f>(BR, RpToTrans(Rwb, ph_BR)));


        Eigen::Vector3f pf_FL {(float)foot_x / 2, (float)foot_y / 2, -(float)height};
        WorldToFoot.insert(std::pair<uint8_t, Eigen::Matrix4f>(FL, RpToTrans(Rwb, pf_FL)));
        
        Eigen::Vector3f pf_FR {(float)foot_x / 2, -(float)foot_y / 2, -(float)height};
        WorldToFoot.insert(std::pair<uint8_t, Eigen::Matrix4f>(FR, RpToTrans(Rwb, pf_FR)));

        Eigen::Vector3f pf_BL {-(float)foot_x / 2, (float)foot_y / 2, -(float)height};
        WorldToFoot.insert(std::pair<uint8_t, Eigen::Matrix4f>(BL, RpToTrans(Rwb, pf_BL)));

        Eigen::Vector3f pf_BR {-(float)foot_x / 2, -(float)foot_y / 2, -(float)height};
        WorldToFoot.insert(std::pair<uint8_t, Eigen::Matrix4f>(BR, RpToTrans(Rwb, pf_BR)));
    }

void QuadModel::HipToFoot(Eigen::Vector3f orn, Eigen::Vector3f pos,
                TransfDict T_bf, std::map<uint8_t, Eigen::Vector3f> *HipToFoot_List)
    {
        Eigen::Matrix3f Rb;
        Eigen::Vector3f dump;
        TransToRp(RPY(orn[0], orn[1], orn[2]), &Rb, &dump);
        
        Eigen::Vector3f pb = pos;
        Eigen::Matrix4f T_wb = RpToTrans(Rb, pb);
        
        for (uint8_t i = 0; i < 4; i++){
            uint8_t key = names[i];
            Eigen::Matrix4f T_wh = WorldToHip[key];

            Eigen::Matrix3f dump2;
            Eigen::Vector3f p_bf;
            TransToRp(T_bf[key], &dump2, &p_bf);

            Eigen::Matrix4f T_bh = TransInv(T_wb) * T_wh;

            Eigen::Matrix3f dump3;
            Eigen::Vector3f p_bh;
            TransToRp(T_bh, &dump3, &p_bh);

            Eigen::Vector3f p_hf0 = p_bf - p_bh;

            Eigen::Matrix4f T_hf = TransInv(T_bh)*T_bf[key];

            Eigen::Matrix3f dump4;
            Eigen::Vector3f p_hf1;
            TransToRp(T_hf, &dump4, &p_hf1);


            if (p_hf1.isZero(0.001f) != p_hf0.isZero(0.001f))
            {
                std::cout << "NOT EQUAL\n";
            }

            (*HipToFoot_List)[key] = p_hf1;
        }
    }

MatrixJoint QuadModel::IK(Eigen::Vector3f orn, Eigen::Vector3f pos,
                        TransfDict T_bf)
    {
        pos[0] += com_offset;
        
        MatrixJoint joint_angles {{0.0f, 0.0f, 0.0f},
                                  {0.0f, 0.0f, 0.0f},
                                  {0.0f, 0.0f, 0.0f},
                                  {0.0f, 0.0f, 0.0f}};

        std::map<uint8_t, Eigen::Vector3f> HipToFoot_List;

        HipToFoot(orn, pos, T_bf, &HipToFoot_List);

        for (uint8_t i = 0; i < 4; i++)
        {
            uint8_t key = names[i];

            Eigen::Vector3f p_hf = HipToFoot_List[key];

            joint_angles.block(i,0,1,3) << Legs.find(key)->second.solveIK(p_hf).transpose();
        } 

        return joint_angles;
    }

MatrixJoint QuadModel::FK(MatrixJoint joint_angles)
    {
        MatrixJoint xyz {{0.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f}};
        
        for (uint8_t i = 0; i < 4; i++)
        {
            Eigen::Vector3f angles = joint_angles.row(i);
            Eigen::Vector3f xyz_coord = Legs.find(names[i])->second.solveFK(angles);
            xyz.block(i,0,1,3) = xyz_coord.transpose();
        }
        
        return xyz;
    }