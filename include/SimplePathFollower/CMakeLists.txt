set(hdrs SimplePathFollower.h
    PARENT_SCOPE
    MobileRobotSVC_impl.h
    SimpleFollower.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

