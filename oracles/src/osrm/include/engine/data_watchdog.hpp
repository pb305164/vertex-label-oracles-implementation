#ifndef OSRM_ENGINE_DATA_WATCHDOG_HPP
#define OSRM_ENGINE_DATA_WATCHDOG_HPP

#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/datafacade/shared_memory_allocator.hpp"
#include "engine/datafacade_factory.hpp"

#include "storage/shared_datatype.hpp"
#include "storage/shared_memory.hpp"
#include "storage/shared_monitor.hpp"

#include <boost/interprocess/sync/named_upgradable_mutex.hpp>
#include <boost/thread/lock_types.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <memory>
#include <thread>

namespace osrm
{
namespace engine
{

namespace detail
{
// We need this wrapper type since template-template specilization of FacadeT is broken on clang
// when it is combined with an templated alias (DataFacade in this case).
// See https://godbolt.org/g/ZS6Xmt for an example.
template <typename AlgorithmT, typename FacadeT> class DataWatchdogImpl;

template <typename AlgorithmT>
class DataWatchdogImpl<AlgorithmT, datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>> final
{
    using mutex_type = typename storage::SharedMonitor<storage::SharedDataTimestamp>::mutex_type;
    using Facade = datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>;

  public:
    DataWatchdogImpl() : active(true), timestamp(0)
    {
        // create the initial facade before launching the watchdog thread
        {
            boost::interprocess::scoped_lock<mutex_type> current_region_lock(barrier.get_mutex());

            facade_factory =
                DataFacadeFactory<datafacade::ContiguousInternalMemoryDataFacade, AlgorithmT>(
                    std::make_shared<datafacade::SharedMemoryAllocator>(barrier.data().region));
            timestamp = barrier.data().timestamp;
        }

        watcher = std::thread(&DataWatchdogImpl::Run, this);
    }

    ~DataWatchdogImpl()
    {
        active = false;
        barrier.notify_all();
        watcher.join();
    }

    std::shared_ptr<const Facade> Get(const api::BaseParameters &params) const
    {
        return facade_factory.Get(params);
    }
    std::shared_ptr<const Facade> Get(const api::TileParameters &params) const
    {
        return facade_factory.Get(params);
    }

  private:
    void Run()
    {
        while (active)
        {
            boost::interprocess::scoped_lock<mutex_type> current_region_lock(barrier.get_mutex());

            while (active && timestamp == barrier.data().timestamp)
            {
                barrier.wait(current_region_lock);
            }

            if (timestamp != barrier.data().timestamp)
            {
                auto region = barrier.data().region;
                facade_factory =
                    DataFacadeFactory<datafacade::ContiguousInternalMemoryDataFacade, AlgorithmT>(
                        std::make_shared<datafacade::SharedMemoryAllocator>(region));
                timestamp = barrier.data().timestamp;
                util::Log() << "updated facade to region " << region << " with timestamp "
                            << timestamp;
            }
        }

        util::Log() << "DataWatchdog thread stopped";
    }

    storage::SharedMonitor<storage::SharedDataTimestamp> barrier;
    std::thread watcher;
    bool active;
    unsigned timestamp;
    DataFacadeFactory<datafacade::ContiguousInternalMemoryDataFacade, AlgorithmT> facade_factory;
};
}

// This class monitors the shared memory region that contains the pointers to
// the data and layout regions that should be used. This region is updated
// once a new dataset arrives.
template <typename AlgorithmT, template <typename A> class FacadeT>
using DataWatchdog = detail::DataWatchdogImpl<AlgorithmT, FacadeT<AlgorithmT>>;
}
}

#endif
