idf_component_register(
    SRCS "main_lqi.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES pwm_ctrl lqi robotica
)
