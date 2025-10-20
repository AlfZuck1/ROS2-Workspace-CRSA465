class CRSA465Config:
    @staticmethod
    def joint_names():
        return [
            "junta_0_1",
            "junta_1_2",
            "junta_2_3",
            "junta_3_4",
            "junta_4_5",
            "junta_5_6"
        ]

    @staticmethod
    def base_link_name():
        return "eslabon_base"

    @staticmethod
    def end_effector_name():
        return "ee_link"
