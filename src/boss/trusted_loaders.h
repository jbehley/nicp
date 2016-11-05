#define BOSS_TRUSTED_LOADER_H(ns) \
    namespace ns { \
    class ns ## _dummy_obj : public nicp::Serializable { \
    public: \
        virtual void serialize(nicp::ObjectData&, nicp::IdContext&); \
        virtual void deserialize(nicp::ObjectData&, nicp::IdContext&); \
    };\
    }

#define BOSS_TRUSTED_LOADER_CPP(ns) \
    namespace ns { \
        void ns ## _dummy_obj::serialize(nicp::ObjectData&, nicp::IdContext&) { } \
        void ns ## _dummy_obj::deserialize(nicp::ObjectData&, nicp::IdContext&) { } \
    BOSS_REGISTER_CLASS(ns ## _dummy_obj); \
    }

// the following MUST BE _dummy_obj_inst and not _dummy_obj_inst() (damned value-initialization "feature")
#define BOSS_ENSURE_LOAD(ns) \
    ns::ns ## _dummy_obj ns ## _dummy_obj_inst;
