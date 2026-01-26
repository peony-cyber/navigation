关于esdf索引的一些说明：
    在esdf_init中传参顺序为map的height、width,对应y、x，与一般x、y索引顺序相反，后续的一些bin_map和distance_map的索引也为y、x；系统逻辑自洽的原因在于连续坐标到栅格坐标变换时改变了索引顺序，所以运行正常。