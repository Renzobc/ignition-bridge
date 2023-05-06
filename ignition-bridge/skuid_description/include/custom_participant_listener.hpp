#include <rmw_fastrtps_shared_cpp/participant.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>


class CustomDomainParticipantListener : public eprosima::fastdds::dds::DomainParticipantListener
{

public:

    CustomDomainParticipantListener()
        : eprosima::fastdds::dds::DomainParticipantListener()
    {
    }

    virtual ~CustomDomainParticipantListener()
    {
    }

    virtual void on_participant_discovery(
            eprosima::fastdds::dds::DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info)
    {
        if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT)
        {
            std::cout << "[ RENZO BRUZZONE ] New participant discovered" << std::endl;
        }
        else if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT ||
                info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT)
        {
            std::cout << "[ RENZO BRUZZONE ] New participant lost" << std::endl;
        }
    }

#if HAVE_SECURITY
    virtual void onParticipantAuthentication(
            eprosima::fastdds::dds::DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ParticipantAuthenticationInfo&& info)
    {
        if (info.status == eprosima::fastrtps::rtps::ParticipantAuthenticationInfo::AUTHORIZED_PARTICIPANT)
        {
            std::cout << "[ RENZO BRUZZONE ] A participant was authorized" << std::endl;
        }
        else if (info.status == eprosima::fastrtps::rtps::ParticipantAuthenticationInfo::UNAUTHORIZED_PARTICIPANT)
        {
            std::cout << "[ RENZO BRUZZONE ] A participant failed authorization" << std::endl;
        }
    }

#endif // if HAVE_SECURITY

    virtual void on_subscriber_discovery(
            eprosima::fastdds::dds::DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::ReaderDiscoveryInfo&& info)
    {
        if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
        {
            std::cout << "[ RENZO BRUZZONE ] New subscriber discovered" << std::endl;
        }
        else if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
        {
            std::cout << "[ RENZO BRUZZONE ] New subscriber lost" << std::endl;
        }
    }

    virtual void on_publisher_discovery(
            eprosima::fastdds::dds::DomainParticipant* /*participant*/,
            eprosima::fastrtps::rtps::WriterDiscoveryInfo&& info)
    {
        if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
        {
            std::cout << " [ RENZO BRUZZONE ] New publisher discovered" << std::endl;
        }
        else if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
        {
            std::cout << " [ RENZO BRUZZONE ] New publisher lost" << std::endl;
        }
    }

    virtual void on_type_discovery(
            eprosima::fastdds::dds::DomainParticipant* participant,
            const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
            const eprosima::fastrtps::string_255& topic,
            const eprosima::fastrtps::types::TypeIdentifier* identifier,
            const eprosima::fastrtps::types::TypeObject* object,
            eprosima::fastrtps::types::DynamicType_ptr dyn_type)
    {
        (void)participant, (void)request_sample_id, (void)topic, (void)identifier, (void)object, (void)dyn_type;
        std::cout << " [ RENZO BRUZZONE ] New data type discovered" << std::endl;

    }

    virtual void on_type_dependencies_reply(
            eprosima::fastdds::dds::DomainParticipant* participant,
            const eprosima::fastrtps::rtps::SampleIdentity& request_sample_id,
            const eprosima::fastrtps::types::TypeIdentifierWithSizeSeq& dependencies)
    {
        (void)participant, (void)request_sample_id, (void)dependencies;
        std::cout << "[ RENZO BRUZZONE ] Answer to a request for type dependencies was received" << std::endl;
    }

    virtual void on_type_information_received(
            eprosima::fastdds::dds::DomainParticipant* participant,
            const eprosima::fastrtps::string_255 topic_name,
            const eprosima::fastrtps::string_255 type_name,
            const eprosima::fastrtps::types::TypeInformation& type_information)
    {
        (void)participant, (void)topic_name, (void)type_name, (void)type_information;
        std::cout << "[ RENZO BRUZZONE ] New data type information received" << std::endl;
    }

};