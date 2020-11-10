insmod gzd-core.ko card_ids=0 media_size=524288 genz_subnets=1 pat=3
insmod gzd-stor.ko bdev_percent=50 num_bdevs=1 num_cdevs=1 pat=3 CHUNK=4096


#pat:
#0 = WB, 1 = WT, 2 = WC, 3 = UC


