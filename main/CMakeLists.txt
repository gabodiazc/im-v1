# SRCS: archivos a compilar.
# INCLUDE_DIRS: directorios donde se buscarán headers. 
# El scope de estos headers será global. 

idf_component_register(SRCS "main.c" 
                            "common/common.c" 
                            "bme688/bme688.c"
                            "bmi270/bmi270.c"
                            "bmm350/bmm350.c"
                            "schema/schema.pb-c.c" 
                       INCLUDE_DIRS "."
                                    "common"
                                    "bme688"
                                    "bmi270"
                                    "bmm350"
                                    "schema"
                                   )

